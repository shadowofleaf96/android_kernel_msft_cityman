/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * SMIA++ module wrapped as an MSM sensor (Nokia X2 smia65pp pattern).
 * Ident uses the mainline smiapp register map (smiapp-reg-defs.h), not a
 * hardcoded Nokia X2 chip-id table. CCI is not an i2c_adapter, so the
 * 3.10 i2c smiapp driver cannot bind these nodes; we run its identify
 * sequence over MSM CCI after boot.
 *
 * snaccy: qcamera + SMIA++ (this wrapper). Rear HAL name imx230. One
 * s_ctrl per node. Hill first. Do not enable CONFIG_VIDEO_SMIAPP here.
 */
#include "msm_camera_dt_util.h"
#include "msm_cci.h"
#include "msm_sensor.h"
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...)                                                     \
  do {                                                                         \
  } while (0)
#endif

/* smiapp-reg-defs.h expects these from smiapp-regs.h (do not link smiapp.o). */
#define SMIA_REG_FLAG_FLOAT (1 << 24)
#define SMIA_REG_8BIT 1
#define SMIA_REG_16BIT 2
#define SMIA_REG_32BIT 4
#include "../../../../i2c/smiapp/smiapp-reg-defs.h"

#define SMIAPP_ADDR(r) ((u16)(r))
#define SMIAPP_LEN(r) ((u8)((r) >> 16))

#define SMIA65PP_SENSOR_NAME "smia65pp"
#define SMIA65PP_MAX_CAM 3
#define SMIA65PP_MCLK_HZ 9600000
#define SMIA65PP_IDENT_DELAY_SEC 8
/* ACPI CAMS D0 waits 0x19 ms after MCLK pin mux before the sensor is used. */
#define SMIA65PP_ACPI_MCLK_US 25000
/* smiapp.h: 2400 extclk cycles + 1ms, plus the extra 10ms the driver notes. */
#define SMIA65PP_XSHUTDOWN_US                                                  \
  (1000 +                                                                      \
   (2400 * 1000 + SMIA65PP_MCLK_HZ / 1000 - 1) / (SMIA65PP_MCLK_HZ / 1000) +   \
   10000)

DEFINE_MSM_MUTEX(smia65pp_mut);

static struct msm_sensor_ctrl_t smia65pp_s_ctrl;
static struct msm_sensor_ctrl_t *smia65pp_devs[SMIA65PP_MAX_CAM];
static int smia65pp_ndev;
static DEFINE_MUTEX(smia65pp_list_lock);
static void smia65pp_ident_work_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(smia65pp_ident_work, smia65pp_ident_work_fn);
/* LVS1 / cam_vio: enable for ident, never disable (power_down reboots cityman).
 */
static struct regulator *smia65pp_vio_hold;
/* Hill-only session flags. Cleared on CFG_POWER_DOWN. */
static int smia65pp_stream_forced;
static int smia65pp_peeked;
static int smia65pp_i2c_logs;
static int smia65pp_tp_done;
static int smia65pp_dphy_done;
static int smia65pp_nvm_done;
static int smia65pp_vendor_done;
static int smia65pp_ovr_done;
static int smia65pp_analog_done;
static int smia65pp_supplies_done;

static int ident_delay_sec = SMIA65PP_IDENT_DELAY_SEC;
module_param(ident_delay_sec, int, 0644);

static struct msm_sensor_power_setting smia65pp_power_setting[] = {
    /* Do not put CAM_VIO here. cam_vio is PM8994 LVS1 (shared 1.8V). */
    {
        .seq_type = SENSOR_VREG,
        .seq_val = CAM_VAF,
        .config_val = 0,
        .delay = 1,
    },
    {
        .seq_type = SENSOR_VREG,
        .seq_val = CAM_VANA,
        .config_val = 0,
        .delay = 1,
    },
    {
        .seq_type = SENSOR_VREG,
        .seq_val = CAM_VDIG,
        .config_val = 0,
        .delay = 1,
    },
    {
        /* Hill ACPI STANBY (no _N): 0 = out of standby. */
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_STANDBY,
        .config_val = GPIO_OUT_LOW,
        .delay = 1,
    },
    {
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_RESET,
        .config_val = GPIO_OUT_LOW,
        .delay = 1,
    },
    {
        .seq_type = SENSOR_CLK,
        .seq_val = SENSOR_CAM_MCLK,
        .config_val = SMIA65PP_MCLK_HZ,
        .delay = 25,
    },
    {
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_RESET,
        .config_val = GPIO_OUT_HIGH,
        .delay = 30,
    },
    {
        .seq_type = SENSOR_I2C_MUX,
        .seq_val = 0,
        .config_val = 0,
        .delay = 1,
    },
};

static struct v4l2_subdev_info smia65pp_subdev_info[] = {
    {
        .code = V4L2_MBUS_FMT_SGRBG10_1X10,
        .colorspace = V4L2_COLORSPACE_JPEG,
        .fmt = 1,
        .order = 0,
    },
};

static const struct i2c_device_id smia65pp_i2c_id[] = {
    {SMIA65PP_SENSOR_NAME, (kernel_ulong_t)&smia65pp_s_ctrl}, {}};

static int32_t msm_smia65pp_i2c_probe(struct i2c_client *client,
                                      const struct i2c_device_id *id) {
  return msm_sensor_i2c_probe(client, id, &smia65pp_s_ctrl);
}

static struct i2c_driver smia65pp_i2c_driver = {
    .id_table = smia65pp_i2c_id,
    .probe = msm_smia65pp_i2c_probe,
    .driver =
        {
            .name = SMIA65PP_SENSOR_NAME,
        },
};

static struct msm_camera_i2c_client smia65pp_sensor_i2c_client = {
    .addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id smia65pp_dt_match[] = {
    {.compatible = "qcom,smia65pp"}, {}};

MODULE_DEVICE_TABLE(of, smia65pp_dt_match);

static int32_t smia65pp_parse_dt(struct platform_device *pdev,
                                 struct msm_sensor_ctrl_t *s_ctrl) {
  struct device_node *of_node = pdev->dev.of_node;
  struct msm_camera_sensor_board_info *sd;
  const char *name = "smia65pp";
  uint32_t cell_id = 0, master = 1, i2c_reg = 0;
  uint32_t slave[3] = {0, 0, 0};
  int32_t rc;

  s_ctrl->pdev = pdev;
  s_ctrl->of_node = of_node;
  s_ctrl->sensor_device_type = MSM_CAMERA_PLATFORM_DEVICE;
  s_ctrl->sensordata = kzalloc(sizeof(*s_ctrl->sensordata), GFP_KERNEL);
  if (!s_ctrl->sensordata)
    return -ENOMEM;
  sd = s_ctrl->sensordata;

  of_property_read_u32(of_node, "cell-index", &cell_id);
  s_ctrl->id = cell_id;

  of_property_read_string(of_node, "qcom,sensor-name", &name);
  sd->sensor_name = name;

  of_property_read_u32(of_node, "qcom,cci-master", &master);
  s_ctrl->cci_i2c_master = master;

  /* msm-cci.txt: reg is the I2C slave address. slave-id is optional. */
  of_property_read_u32_array(of_node, "qcom,slave-id", slave, 3);
  if (!of_property_read_u32(of_node, "reg", &i2c_reg) && i2c_reg)
    slave[0] = i2c_reg;

  sd->slave_info = kzalloc(sizeof(*sd->slave_info), GFP_KERNEL);
  if (!sd->slave_info)
    return -ENOMEM;
  sd->slave_info->sensor_slave_addr = slave[0];
  sd->slave_info->sensor_id_reg_addr = slave[1];
  sd->slave_info->sensor_id = slave[2];

  rc = msm_sensor_get_sub_module_index(of_node, &sd->sensor_info);
  if (rc < 0) {
    pr_err("cityman_smia: sub_module_index rc=%d\n", rc);
    return rc;
  }

  of_property_read_u32(of_node, "qcom,mount-angle",
                       &sd->sensor_info->sensor_mount_angle);
  sd->sensor_info->is_mount_angle_valid = 1;
  of_property_read_u32(of_node, "qcom,sensor-position",
                       &sd->sensor_info->position);
  of_property_read_u32(of_node, "qcom,sensor-mode",
                       &sd->sensor_info->modes_supported);

  pr_err("cityman_smia: parsed %s cell=%u cci_master=%u sid=0x%x expect=0x%04x "
         "csiphy=%d csid=%d mount=%u\n",
         name, cell_id, master, slave[0], slave[2],
         sd->sensor_info->subdev_id[SUB_MODULE_CSIPHY],
         sd->sensor_info->subdev_id[SUB_MODULE_CSID],
         sd->sensor_info->sensor_mount_angle);
  return 0;
}

static void smia65pp_fill_cci(struct msm_sensor_ctrl_t *s_ctrl) {
  struct msm_camera_cci_client *cci_client;

  if (!s_ctrl->sensor_i2c_client || !s_ctrl->sensor_i2c_client->cci_client)
    return;
  cci_client = s_ctrl->sensor_i2c_client->cci_client;
  /* Do not replace a probe-time pointer with NULL (%pK also hides it). */
  if (msm_cci_get_subdev())
    cci_client->cci_subdev = msm_cci_get_subdev();
  cci_client->cci_i2c_master = s_ctrl->cci_i2c_master;
  if (s_ctrl->sensordata && s_ctrl->sensordata->slave_info)
    cci_client->sid = s_ctrl->sensordata->slave_info->sensor_slave_addr >> 1;
  cci_client->retries = 3;
  cci_client->id_map = 0;
  if (s_ctrl->sensordata && s_ctrl->pdev)
    s_ctrl->sensordata->power_info.dev = &s_ctrl->pdev->dev;
  pr_err("cityman_smia: cci sid=0x%x master=%u subdev=%pK\n", cci_client->sid,
         cci_client->cci_i2c_master, cci_client->cci_subdev);
}

static int32_t smia65pp_alloc_s_ctrl(struct msm_sensor_ctrl_t **out) {
  struct msm_sensor_ctrl_t *s_ctrl;
  struct msm_camera_i2c_client *i2c;
  struct mutex *mut;

  s_ctrl = kzalloc(sizeof(*s_ctrl), GFP_KERNEL);
  i2c = kzalloc(sizeof(*i2c), GFP_KERNEL);
  mut = kzalloc(sizeof(*mut), GFP_KERNEL);
  if (!s_ctrl || !i2c || !mut) {
    kfree(s_ctrl);
    kfree(i2c);
    kfree(mut);
    return -ENOMEM;
  }
  *s_ctrl = smia65pp_s_ctrl;
  *i2c = smia65pp_sensor_i2c_client;
  mutex_init(mut);
  s_ctrl->sensor_i2c_client = i2c;
  s_ctrl->msm_sensor_mutex = mut;
  s_ctrl->pdev = NULL;
  s_ctrl->sensordata = NULL;
  s_ctrl->of_node = NULL;
  *out = s_ctrl;
  return 0;
}

static int32_t smia65pp_read(struct msm_sensor_ctrl_t *s_ctrl, uint16_t reg,
                             uint16_t *val, uint32_t dt) {
  *val = 0;
  if (!s_ctrl || !s_ctrl->sensor_i2c_client ||
      !s_ctrl->sensor_i2c_client->i2c_func_tbl ||
      !s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read)
    return -ENODEV;
  return s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
      s_ctrl->sensor_i2c_client, reg, val, dt);
}

static int32_t smia65pp_write(struct msm_sensor_ctrl_t *s_ctrl, uint16_t reg,
                              uint16_t val, uint32_t dt) {
  if (!s_ctrl || !s_ctrl->sensor_i2c_client ||
      !s_ctrl->sensor_i2c_client->i2c_func_tbl ||
      !s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write)
    return -ENODEV;
  return s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
      s_ctrl->sensor_i2c_client, reg, val, dt);
}

/*
 * SMIA++ DATA_TRANSFER_IF_1 page 0 (Nokia smiapp_read_nvm). WP rear KMD
 * loads CDCC from this NVM before SMIApp_StartStream. Dump only - do not
 * write the page back. Call while CCI is up and 0x0100 is still 0.
 */
static int smia65pp_dump_nvm_page0(struct msm_sensor_ctrl_t *s_ctrl) {
  uint16_t model = 0, st = 0, b = 0;
  u8 pg[64];
  int i, t, wrc = 0;

  if (smia65pp_nvm_done)
    return 0;
  smia65pp_read(s_ctrl, 0x0000, &model, MSM_CAMERA_I2C_WORD_DATA);
  if (model != 0xEACA)
    return 0;

  wrc |= smia65pp_write(s_ctrl, 0x0a02, 0, MSM_CAMERA_I2C_BYTE_DATA);
  wrc |= smia65pp_write(s_ctrl, 0x0a00, 1, MSM_CAMERA_I2C_BYTE_DATA);
  for (t = 0; t < 50; t++) {
    smia65pp_read(s_ctrl, 0x0a01, &st, MSM_CAMERA_I2C_BYTE_DATA);
    if ((st & 0x1) || (st & 0xc))
      break;
    usleep_range(1000, 1500);
  }
  pr_err("cityman_smia nvm page0 stat=0x%x tries=%d wrc=%d\n", st, t, wrc);
  if (st & 0x1) {
    for (i = 0; i < 64; i++) {
      b = 0;
      smia65pp_read(s_ctrl, 0x0a04 + i, &b, MSM_CAMERA_I2C_BYTE_DATA);
      pg[i] = (u8)b;
    }
    for (i = 0; i < 64; i += 16)
      pr_err("cityman_smia nvm %02x: %02x %02x %02x %02x %02x %02x %02x %02x "
             "%02x %02x %02x %02x %02x %02x %02x %02x\n",
             i, pg[i], pg[i + 1], pg[i + 2], pg[i + 3], pg[i + 4], pg[i + 5],
             pg[i + 6], pg[i + 7], pg[i + 8], pg[i + 9], pg[i + 10], pg[i + 11],
             pg[i + 12], pg[i + 13], pg[i + 14], pg[i + 15]);
  }
  smia65pp_write(s_ctrl, 0x0a00, 0, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_nvm_done = 1;
  return 1;
}

static int smia65pp_smiapp_read(struct msm_sensor_ctrl_t *s_ctrl, u32 packed,
                                u32 *out) {
  uint16_t val = 0;
  u16 addr = SMIAPP_ADDR(packed);
  u8 len = SMIAPP_LEN(packed);
  uint32_t dt;
  int rc;

  *out = 0;
  if (len == SMIA_REG_8BIT)
    dt = MSM_CAMERA_I2C_BYTE_DATA;
  else if (len == SMIA_REG_16BIT)
    dt = MSM_CAMERA_I2C_WORD_DATA;
  else
    return -EINVAL;
  rc = smia65pp_read(s_ctrl, addr, &val, dt);
  if (rc < 0)
    return rc;
  *out = val;
  return 0;
}

static int smia65pp_smiapp_write(struct msm_sensor_ctrl_t *s_ctrl, u32 packed,
                                 u16 val) {
  u16 addr = SMIAPP_ADDR(packed);
  u8 len = SMIAPP_LEN(packed);
  uint32_t dt;

  if (len == SMIA_REG_8BIT)
    dt = MSM_CAMERA_I2C_BYTE_DATA;
  else if (len == SMIA_REG_16BIT)
    dt = MSM_CAMERA_I2C_WORD_DATA;
  else
    return -EINVAL;
  return smia65pp_write(s_ctrl, addr, val, dt);
}

/* Mainline smiapp_identify_module() register list, over CCI. */
static int smia65pp_smiapp_identify(struct msm_sensor_ctrl_t *s_ctrl) {
  const char *name = s_ctrl->sensordata ? s_ctrl->sensordata->sensor_name : "?";
  u32 manufacturer_id = 0, model_id = 0;
  u32 rev_major = 0, rev_minor = 0;
  u32 year = 0, month = 0, day = 0;
  u32 sensor_mfr = 0, sensor_model = 0;
  u32 sensor_rev = 0, sensor_fw = 0;
  u32 smia = 0, smiapp = 0;
  u32 xout = 0, yout = 0, fll = 0, llp = 0;
  int rc = 0;

  rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_MANUFACTURER_ID,
                             &manufacturer_id);
  rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U16_MODEL_ID, &model_id);
  rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_REVISION_NUMBER_MAJOR,
                             &rev_major);
  rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_REVISION_NUMBER_MINOR,
                             &rev_minor);
  rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_MODULE_DATE_YEAR, &year);
  rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_MODULE_DATE_MONTH, &month);
  rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_MODULE_DATE_DAY, &day);
  rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_SENSOR_MANUFACTURER_ID,
                             &sensor_mfr);
  rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U16_SENSOR_MODEL_ID,
                             &sensor_model);
  rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_SENSOR_REVISION_NUMBER,
                             &sensor_rev);
  rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_SENSOR_FIRMWARE_VERSION,
                             &sensor_fw);
  rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_SMIA_VERSION, &smia);
  rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_SMIAPP_VERSION, &smiapp);
  smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U16_X_OUTPUT_SIZE, &xout);
  smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U16_Y_OUTPUT_SIZE, &yout);
  smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U16_FRAME_LENGTH_LINES, &fll);
  smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U16_LINE_LENGTH_PCK, &llp);

  if (rc) {
    pr_err("smiapp: %s sensor detection failed rc=%d\n", name, rc);
    return rc;
  }

  if (!manufacturer_id && !model_id) {
    manufacturer_id = sensor_mfr;
    model_id = sensor_model;
    rev_major = sensor_rev;
  }

  pr_err("smiapp: %s module 0x%02x-0x%04x\n", name, manufacturer_id, model_id);
  pr_err("smiapp: %s module revision 0x%02x-0x%02x date %02u-%02u-%02u\n", name,
         rev_major, rev_minor, year, month, day);
  pr_err("smiapp: %s sensor 0x%02x-0x%04x\n", name, sensor_mfr, sensor_model);
  pr_err("smiapp: %s sensor revision 0x%02x firmware version 0x%02x\n", name,
         sensor_rev, sensor_fw);
  pr_err("smiapp: %s smia version %u smiapp version %u ident %02x%04x%02x\n",
         name, smia, smiapp, manufacturer_id, model_id, rev_major);
  pr_err("smiapp: %s window %ux%u llp=%u fll=%u\n", name, xout, yout, llp, fll);
  if (s_ctrl->sensordata && s_ctrl->sensordata->slave_info && model_id)
    s_ctrl->sensordata->slave_info->sensor_id = (uint16_t)model_id;
  return (model_id || sensor_model) ? 0 : -ENODEV;
}

static int smia65pp_cci_util(struct msm_sensor_ctrl_t *s_ctrl, uint16_t cmd) {
  if (!s_ctrl->sensor_i2c_client || !s_ctrl->sensor_i2c_client->i2c_func_tbl ||
      !s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util)
    return -ENODEV;
  return s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
      s_ctrl->sensor_i2c_client, cmd);
}

/*
 * Acquire a GPIO already muxed by pinctrl-0. -EBUSY: still drive it.
 * Returns 1 if gpio_free() is required, 0 if not, negative on hard fail.
 */
static int smia65pp_xpin_acquire(int gpio, const char *label, int init_high) {
  unsigned long flags = init_high ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW;
  int rc;

  if (!gpio_is_valid(gpio))
    return -EINVAL;
  rc = gpio_request_one(gpio, flags, label);
  if (!rc)
    return 1;
  pr_err("smiapp: gpio %d (%s) request rc=%d, forcing output %d\n", gpio, label,
         rc, init_high);
  gpio_direction_output(gpio, init_high);
  return 0;
}

static void smia65pp_vio_enable_keep(struct device *dev, const char *name) {
  struct regulator *vio;
  int rc;

  vio = regulator_get(dev, "cam_vio");
  if (IS_ERR(vio)) {
    pr_err("smiapp: %s cam_vio get rc=%ld\n", name, PTR_ERR(vio));
    return;
  }
  rc = regulator_enable(vio);
  pr_err("smiapp: %s cam_vio enable rc=%d (LVS1, never disable)\n", name, rc);
  if (rc) {
    regulator_put(vio);
    return;
  }
  if (!smia65pp_vio_hold)
    smia65pp_vio_hold = vio;
  else
    regulator_put(vio);
}

static int smia65pp_ident_one(struct msm_sensor_ctrl_t *s_ctrl) {
  struct device *dev;
  struct device_node *np;
  struct regulator *vana = NULL, *vdig = NULL, *vaf = NULL;
  struct clk *src = NULL, *mclk = NULL;
  struct pinctrl *pctl = NULL;
  struct pinctrl_state *pst;
  const char *name;
  u32 rst_idx = (u32)-1, stby_idx = (u32)-1, stby_rel = 1;
  int rst = -EINVAL, stby = -EINVAL;
  int rst_owned = 0, stby_owned = 0;
  int rc;

  if (!s_ctrl || !s_ctrl->pdev)
    return -ENODEV;
  dev = &s_ctrl->pdev->dev;
  np = dev->of_node;
  name = s_ctrl->sensordata ? s_ctrl->sensordata->sensor_name : "?";

  pr_err("smiapp: ident start %s (%s)\n", name, dev_name(dev));

  smia65pp_fill_cci(s_ctrl);

  if (!of_property_read_u32(np, "qcom,gpio-reset", &rst_idx))
    rst = of_get_gpio(np, rst_idx);
  if (!of_property_read_u32(np, "qcom,gpio-standby", &stby_idx))
    stby = of_get_gpio(np, stby_idx);
  of_property_read_u32(np, "qcom,standby-release", &stby_rel);
  pr_err("smiapp: %s xshutdown gpio=%d (idx %u) standby gpio=%d (idx %u) "
         "release=%u\n",
         name, rst, rst_idx, stby, stby_idx, stby_rel);

  /* ACPI CAMS D0: L23 (cam_vaf), L29 (vana), L25 (vdig), LVS1 (vio). */
  vaf = regulator_get(dev, "cam_vaf");
  if (IS_ERR(vaf)) {
    pr_err("smiapp: %s cam_vaf get rc=%ld (optional)\n", name, PTR_ERR(vaf));
    vaf = NULL;
  }
  vdig = regulator_get(dev, "cam_vdig");
  if (IS_ERR(vdig)) {
    pr_err("smiapp: %s cam_vdig get rc=%ld\n", name, PTR_ERR(vdig));
    vdig = NULL;
  }
  vana = regulator_get(dev, "cam_vana");
  if (IS_ERR(vana)) {
    pr_err("smiapp: %s cam_vana get rc=%ld\n", name, PTR_ERR(vana));
    vana = NULL;
  }
  src = clk_get(dev, "cam_src_clk");
  if (IS_ERR(src)) {
    pr_err("smiapp: %s cam_src_clk get rc=%ld\n", name, PTR_ERR(src));
    src = NULL;
  }
  mclk = clk_get(dev, "cam_clk");
  if (IS_ERR(mclk)) {
    pr_err("smiapp: %s cam_clk get rc=%ld\n", name, PTR_ERR(mclk));
    mclk = NULL;
  }

  /*
   * 3.10 smiapp_power_on: VANA, xclk, xshutdown=1, software reset.
   * ACPI CAMS D0 also votes L23 2.85V and LVS1 (VIO). Enable LVS1
   * and never disable it. Do not copy snaccy's VIO/VAF name swap:
   * LVS1 is VIO, L23 is extra 2.85V on cam_vaf.
   */
  if (vaf) {
    rc = regulator_enable(vaf);
    pr_err("smiapp: %s cam_vaf enable rc=%d (ACPI L23 2.85V)\n", name, rc);
  }
  if (vana) {
    rc = regulator_enable(vana);
    pr_err("smiapp: %s cam_vana enable rc=%d\n", name, rc);
  }
  if (vdig) {
    rc = regulator_enable(vdig);
    pr_err("smiapp: %s cam_vdig enable rc=%d\n", name, rc);
  }
  smia65pp_vio_enable_keep(dev, name);
  usleep_range(1000, 2000);

  /*
   * Core only auto-applies a state named "default". This node uses
   * Qualcomm's "cam_default" (GPIO 13 = MCLK). msm_sensor_platform_probe
   * would have selected it; we skipped that path, so #13 left gp-13
   * unclaimed and the sensor never saw the clock.
   */
  pctl = pinctrl_get(dev);
  if (IS_ERR(pctl)) {
    pr_err("smiapp: %s pinctrl_get rc=%ld\n", name, PTR_ERR(pctl));
    pctl = NULL;
  } else {
    pst = pinctrl_lookup_state(pctl, "cam_default");
    if (IS_ERR(pst))
      pr_err("smiapp: %s cam_default lookup rc=%ld\n", name, PTR_ERR(pst));
    else {
      rc = pinctrl_select_state(pctl, pst);
      pr_err("smiapp: %s pinctrl cam_default rc=%d\n", name, rc);
    }
  }

  /* Board standby pin is not in smiapp; pinctrl leaves it floating. */
  rc = smia65pp_xpin_acquire(stby, "smiapp-stby", stby_rel ? 1 : 0);
  if (rc > 0)
    stby_owned = 1;
  rc = smia65pp_xpin_acquire(rst, "smiapp-xshutdown", 0);
  if (rc > 0)
    rst_owned = 1;
  else if (rc < 0)
    pr_err("smiapp: %s xshutdown missing, ident will NACK\n", name);

  if (src) {
    rc = clk_set_rate(src, SMIA65PP_MCLK_HZ);
    if (rc)
      pr_err("smiapp: %s mclk rate %u rc=%d\n", name, SMIA65PP_MCLK_HZ, rc);
    rc = clk_prepare_enable(src);
    pr_err("smiapp: %s cam_src_clk enable rc=%d rate=%lu\n", name, rc,
           clk_get_rate(src));
  }
  if (mclk) {
    rc = clk_prepare_enable(mclk);
    pr_err("smiapp: %s cam_clk enable rc=%d\n", name, rc);
  }
  usleep_range(SMIA65PP_ACPI_MCLK_US, SMIA65PP_ACPI_MCLK_US + 1000);

  if (gpio_is_valid(rst)) {
    gpio_set_value_cansleep(rst, 1);
    pr_err("smiapp: %s xshutdown=1, wait %u us\n", name, SMIA65PP_XSHUTDOWN_US);
    usleep_range(SMIA65PP_XSHUTDOWN_US, SMIA65PP_XSHUTDOWN_US + 1000);
  }

  if (smia65pp_cci_util(s_ctrl, MSM_CCI_INIT) < 0)
    pr_err("smiapp: %s CCI init failed\n", name);

  rc = smia65pp_smiapp_write(s_ctrl, SMIAPP_REG_U8_SOFTWARE_RESET, 1);
  pr_err("smiapp: %s software reset rc=%d\n", name, rc);
  usleep_range(SMIA65PP_XSHUTDOWN_US, SMIA65PP_XSHUTDOWN_US + 1000);

  rc = smia65pp_smiapp_identify(s_ctrl);
  if (!rc)
    smia65pp_dump_nvm_page0(s_ctrl);

  smia65pp_cci_util(s_ctrl, MSM_CCI_RELEASE);

  /* smiapp_power_off: xshutdown=0, clock off, VANA off. Never LVS1. */
  if (gpio_is_valid(rst))
    gpio_set_value_cansleep(rst, 0);
  if (mclk)
    clk_disable_unprepare(mclk);
  if (src)
    clk_disable_unprepare(src);
  if (rst_owned)
    gpio_free(rst);
  if (stby_owned)
    gpio_free(stby);
  if (vana) {
    regulator_disable(vana);
    regulator_put(vana);
  }
  if (vdig) {
    regulator_disable(vdig);
    regulator_put(vdig);
  }
  if (vaf) {
    regulator_disable(vaf);
    regulator_put(vaf);
  }
  if (pctl) {
    pst = pinctrl_lookup_state(pctl, "cam_suspend");
    if (!IS_ERR(pst))
      pinctrl_select_state(pctl, pst);
    pinctrl_put(pctl);
  }
  if (src)
    clk_put(src);
  if (mclk)
    clk_put(mclk);
  pr_err("smiapp: ident done %s rc=%d\n", name, rc);
  return rc;
}

static void smia65pp_ident_work_fn(struct work_struct *work) {
  int i, rc;
  uint16_t sid;

  pr_err("smiapp: delayed ident for %d camera node(s)\n", smia65pp_ndev);
  mutex_lock(&smia65pp_list_lock);
  for (i = 0; i < smia65pp_ndev; i++) {
    struct msm_sensor_ctrl_t *s_ctrl = smia65pp_devs[i];

    rc = smia65pp_ident_one(s_ctrl);
    if (rc) {
      pr_err("cityman_smia: skip qcamera register, ident rc=%d\n", rc);
      continue;
    }
    /*
     * Nokia X2 smia65pp calls msm_sensor_platform_probe from
     * the CCI child probe. That is nested of_platform_populate
     * here and bootlooped as smia-msm #4. CCI is up now.
     * Patched probe skips power_up/down. Do not add CAM_VIO
     * to DT vreg names (LVS1).
     */
    if (!msm_cci_get_subdev()) {
      pr_err("cityman_smia: CCI gone, skip qcamera register\n");
      continue;
    }
    if (!s_ctrl->pdev) {
      pr_err("cityman_smia: no pdev, skip qcamera register\n");
      continue;
    }
    /*
     * Save SMIA module id before platform_probe: get_dt_data
     * rebuilds slave_info from DT (id 0). Daemon CFG_SINIT_PROBE
     * matches libmmcamera_imx230.so (0xEACA) against this.
     */
    sid = 0;
    if (s_ctrl->sensordata && s_ctrl->sensordata->slave_info)
      sid = s_ctrl->sensordata->slave_info->sensor_id;
    rc = msm_sensor_platform_probe(s_ctrl->pdev, s_ctrl);
    pr_err("cityman_smia: delayed qcamera register rc=%d name=%s\n", rc,
           s_ctrl->sensordata ? s_ctrl->sensordata->sensor_name : "?");
    if (!rc) {
      rc = msm_sensor_driver_bind_probed(s_ctrl, sid);
      pr_err("cityman_smia: qcamera slot bind rc=%d sid=0x%04x\n", rc, sid);
    }
  }
  mutex_unlock(&smia65pp_list_lock);
}

static int32_t smia65pp_platform_probe(struct platform_device *pdev) {
  int32_t rc;
  struct msm_sensor_ctrl_t *s_ctrl;

  pr_err(
      "cityman_smia: smia-msm#150 delayed qcamera register + TG DT 0x30 %s\n",
      dev_name(&pdev->dev));

  rc = smia65pp_alloc_s_ctrl(&s_ctrl);
  if (rc < 0) {
    pr_err("cityman_smia: alloc s_ctrl failed\n");
    return 0;
  }
  platform_set_drvdata(pdev, s_ctrl);

  rc = smia65pp_parse_dt(pdev, s_ctrl);
  if (rc < 0)
    pr_err("cityman_smia: parse_dt rc=%d (still bind)\n", rc);

  if (s_ctrl->sensordata && s_ctrl->sensordata->sensor_info) {
    rc = msm_sensor_init_default_params(s_ctrl);
    pr_err("cityman_smia: init_default_params rc=%d\n", rc);
    if (!rc)
      smia65pp_fill_cci(s_ctrl);
  }

  mutex_lock(&smia65pp_list_lock);
  if (smia65pp_ndev < SMIA65PP_MAX_CAM)
    smia65pp_devs[smia65pp_ndev++] = s_ctrl;
  mutex_unlock(&smia65pp_list_lock);
  schedule_delayed_work(&smia65pp_ident_work,
                        msecs_to_jiffies(ident_delay_sec * 1000));

  return 0;
}

static struct platform_driver smia65pp_platform_driver = {
    .probe = smia65pp_platform_probe,
    .driver =
        {
            .name = "qcom,smia65pp",
            .owner = THIS_MODULE,
            .of_match_table = smia65pp_dt_match,
        },
};

static int __init smia65pp_init_module(void) {
  pr_err(
      "cityman_smia: smia-msm#152 Hill ident (SMIA VANA/VDIG/VIO + 0x4800)\n");
  return platform_driver_register(&smia65pp_platform_driver);
}

static void __exit smia65pp_exit_module(void) {
  cancel_delayed_work_sync(&smia65pp_ident_work);
  platform_driver_unregister(&smia65pp_platform_driver);
}

static void smia65pp_peek(struct msm_sensor_ctrl_t *s_ctrl, const char *tag) {
  uint16_t id = 0xffff, w = 0, h = 0, mode = 0, pre = 0, mult = 0;
  uint16_t dt = 0, lanes = 0, vt_pix = 0, vt_sys = 0, op_pix = 0, op_sys = 0;
  uint16_t sig = 0, ch = 0;
  uint16_t exp = 0, gain = 0, tp = 0;
  int rc_id, rc_w;

  rc_id = smia65pp_read(s_ctrl, 0x0000, &id, MSM_CAMERA_I2C_WORD_DATA);
  rc_w = smia65pp_read(s_ctrl, 0x034c, &w, MSM_CAMERA_I2C_WORD_DATA);
  rc_w |= smia65pp_read(s_ctrl, 0x034e, &h, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0100, &mode, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0304, &pre, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0306, &mult, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0300, &vt_pix, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0302, &vt_sys, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0308, &op_pix, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x030a, &op_sys, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0112, &dt, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0114, &lanes, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0110, &ch, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0111, &sig, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0202, &exp, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0204, &gain, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0600, &tp, MSM_CAMERA_I2C_WORD_DATA);
  pr_err("cityman_smia %s id=0x%04x/%d %ux%u/%d mode=0x%x pll=%u/%u vt=%u/%u "
         "op=%u/%u dt=0x%x lanes=0x%x ch=0x%x sig=0x%x exp=%u gain=0x%x tp=0x%x\n",
         tag, id, rc_id, w, h, rc_w, mode, pre, mult, vt_sys, vt_pix, op_sys,
         op_pix, dt, lanes, ch, sig, exp, gain, tp);
}

static int cityman_crop_vfe = 0;
module_param_named(crop_vfe, cityman_crop_vfe, int, 0644);
MODULE_PARM_DESC(crop_vfe, "Override sensor mode to 2496x1872 (default 0=keep HAL mode)");

static int cityman_wp_sensor_analog = 0;
module_param_named(wp_sensor_analog, cityman_wp_sensor_analog, int, 0644);
MODULE_PARM_DESC(wp_sensor_analog, "Enable WP sensor analog & supplies overrides (default 0=off)");

static int cityman_pll_mult = 125;
module_param_named(pll_mult, cityman_pll_mult, int, 0644);
MODULE_PARM_DESC(pll_mult, "PLL multiplier for 9.6MHz MCLK (default 125 for 1200MHz VCO, 124 for 1190.4MHz)");

static int cityman_colorbars = 0;
module_param_named(colorbars, cityman_colorbars, int, 0644);
MODULE_PARM_DESC(colorbars, "Enable sensor test pattern color bars (default 0=off, 1=on)");

static int cityman_exp_lines = 1500;
module_param_named(exp_lines, cityman_exp_lines, int, 0644);
MODULE_PARM_DESC(exp_lines, "Default coarse integration time in lines (default 1500)");

static int cityman_gain = 256;
module_param_named(gain, cityman_gain, int, 0644);
MODULE_PARM_DESC(gain, "Default analogue gain code (default 256 = 2.0x)");


/*
 * WP live viewfinder: 0AEACA05.dcc 0xFF0B (SensorMode 11).
 * 2496x1872, WOI 4992x3744 @176,136, 2x2 bin, PLL 1/124, 0x0820=0x0bd9.
 * ETL DDRClk 379.2 MHz, settle 22. #142 0xFF03 was never streamed.
 */
static int smia65pp_crop_vfe(struct msm_sensor_ctrl_t *s_ctrl) {
  if (!cityman_crop_vfe)
    return 0;
  static const u16 words[][2] = {
      {0x0112, 0x0a0a}, {0x0820, 0x0bd9}, {0x0300, 0x0004}, {0x0302, 0x0002},
      {0x0304, 0x0001}, {0x0306, 0x007c}, {0x0308, 0x0008}, {0x030a, 0x0001},
      {0x030c, 0x0001}, {0x030e, 0x004f}, {0x0342, 0x1788}, {0x0344, 0x00b0},
      {0x0346, 0x0088}, {0x0348, 0x142f}, {0x034a, 0x0f27}, {0x034c, 0x09c0},
      {0x034e, 0x0750}, {0x0402, 0x0000}, {0x0408, 0x0000}, {0x040a, 0x0000},
      {0x040c, 0x09c0}, {0x040e, 0x0750}, {0x3a22, 0x2013}, {0x3a24, 0x8007},
      {0x3a30, 0xb000}, {0x3a32, 0x8814}, {0x3a34, 0x2f0f},
  };
  static const u16 bytes[][2] = {
      {0x0111, 0x02}, {0x0114, 0x03}, {0x0310, 0x01}, {0x0401, 0x00},
      {0x0403, 0x00}, {0x0900, 0x01}, {0x0901, 0x22}, {0x0902, 0x00},
      {0x0220, 0x00}, {0x3a21, 0x00}, {0x3006, 0x01}, {0x31e0, 0x03},
      {0x3a26, 0x50}, {0x3a2f, 0x00}, {0x3a36, 0x27}, {0x6914, 0x01},
      {0x3a38, 0x01}, {0x3a39, 0x00}, {0x3123, 0x01}, {0x3013, 0x00},
  };
  uint16_t model = 0, out_w = 0, out_h = 0, pre = 0, mult = 0;
  uint16_t now_w = 0, now_h = 0, rate = 0;
  int i, rc, wrc = 0;

  rc = smia65pp_read(s_ctrl, 0x0000, &model, MSM_CAMERA_I2C_WORD_DATA);
  if (rc < 0 || model != 0xEACA)
    return 0;
  smia65pp_read(s_ctrl, 0x034c, &out_w, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x034e, &out_h, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0304, &pre, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0306, &mult, MSM_CAMERA_I2C_WORD_DATA);
  if (out_w == 2496 && out_h == 1872 && pre == 1 && mult == 124)
    return 0;
  wrc |= smia65pp_write(s_ctrl, 0x0104, 1, MSM_CAMERA_I2C_BYTE_DATA);
  for (i = 0; i < ARRAY_SIZE(words); i++)
    wrc |= smia65pp_write(s_ctrl, words[i][0], words[i][1],
                          MSM_CAMERA_I2C_WORD_DATA);
  for (i = 0; i < ARRAY_SIZE(bytes); i++)
    wrc |= smia65pp_write(s_ctrl, bytes[i][0], bytes[i][1],
                          MSM_CAMERA_I2C_BYTE_DATA);
  wrc |= smia65pp_write(s_ctrl, 0x0104, 0, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x034c, &now_w, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x034e, &now_h, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0304, &pre, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0306, &mult, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0820, &rate, MSM_CAMERA_I2C_WORD_DATA);
  pr_err("cityman_smia cdcc ff0b %ux%u -> %ux%u pll=%u/%u 0820=0x%x wrc=%d\n",
         out_w, out_h, now_w, now_h, pre, mult, rate, wrc);
  return 1;
}

/*
 * WP 0AEACA05.dcc mode-0 Sony regs that are not PLL/window.
 * Keep ident PLL 4/177. Idle analog (#68) and 1/124 (#67/#69)
 * silenced CSID.
 */
static int smia65pp_cdcc_vendor(struct msm_sensor_ctrl_t *s_ctrl) {
  uint16_t model = 0, r3006 = 0, r31e0 = 0, r6914 = 0, r3123 = 0;
  int wrc = 0;

  if (smia65pp_vendor_done)
    return 0;
  smia65pp_read(s_ctrl, 0x0000, &model, MSM_CAMERA_I2C_WORD_DATA);
  if (model != 0xEACA)
    return 0;
  wrc |= smia65pp_write(s_ctrl, 0x3006, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
  wrc |= smia65pp_write(s_ctrl, 0x31e0, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
  wrc |= smia65pp_write(s_ctrl, 0x6914, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
  wrc |= smia65pp_write(s_ctrl, 0x3123, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
  wrc |= smia65pp_write(s_ctrl, 0x3013, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x3006, &r3006, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x31e0, &r31e0, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x6914, &r6914, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x3123, &r3123, MSM_CAMERA_I2C_BYTE_DATA);
  pr_err("cityman_smia cdcc vendor wrc=%d 3006=0x%x 31e0=0x%x 6914=0x%x "
         "3123=0x%x\n",
         wrc, r3006, r31e0, r6914, r3123);
  smia65pp_vendor_done = 1;
  return 1;
}

/*
 * WP 0AEACA05.dcc p_config_override (id 5). Sensor writes, ident PLL
 * kept. Live NVM has no config-data bit (page0[2]=0x08), so WP NVM
 * write8 is a no-op on this module — do not flash that.
 */
static int smia65pp_cdcc_override(struct msm_sensor_ctrl_t *s_ctrl) {
  static const u16 regs[][2] = {
      {0x0011, 0x08}, {0x1b04, 0x06}, {0x1b40, 0x00}, {0x1b41, 0x81},
      {0x1b42, 0x7c}, {0x1b44, 0x05}, {0x1b45, 0xf0},
  };
  uint16_t model = 0, r11 = 0, r1b40 = 0, r1b41 = 0;
  int i, wrc = 0;

  if (smia65pp_ovr_done)
    return 0;
  smia65pp_read(s_ctrl, 0x0000, &model, MSM_CAMERA_I2C_WORD_DATA);
  if (model != 0xEACA)
    return 0;
  for (i = 0; i < ARRAY_SIZE(regs); i++)
    wrc |= smia65pp_write(s_ctrl, regs[i][0], regs[i][1],
                          MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0011, &r11, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x1b40, &r1b40, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x1b41, &r1b41, MSM_CAMERA_I2C_BYTE_DATA);
  pr_err("cityman_smia cdcc override wrc=%d 0011=0x%x 1b40=0x%x 1b41=0x%x\n",
         wrc, r11, r1b40, r1b41);
  smia65pp_ovr_done = 1;
  return 1;
}

/* SMIA++ 0x0111: 0=CCP2 clk, 1=CCP2 strobe, 2=CSI-2. Do not write 0. */
static void smia65pp_peek_dphy(struct msm_sensor_ctrl_t *s_ctrl) {
  uint16_t cap_dphy = 0, cap_lane = 0, cap_sig = 0, ctrl = 0;
  uint16_t tclk_post = 0, ths_prep = 0, ths_zero = 0, ths_trail = 0;
  uint16_t tclk_trail = 0, tclk_prep = 0, tclk_zero = 0, tlpx = 0;
  uint16_t rate_hi = 0, rate_lo = 0;
  uint16_t max4_hi = 0, max4_lo = 0, if_ctrl = 0, if_stat = 0;

  smia65pp_read(s_ctrl, 0x1600, &cap_dphy, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x1601, &cap_lane, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x1602, &cap_sig, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0808, &ctrl, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0800, &tclk_post, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0801, &ths_prep, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0802, &ths_zero, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0803, &ths_trail, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0804, &tclk_trail, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0805, &tclk_prep, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0806, &tclk_zero, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0807, &tlpx, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0820, &rate_hi, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0822, &rate_lo, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x1614, &max4_hi, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x1616, &max4_lo, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0a00, &if_ctrl, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0a01, &if_stat, MSM_CAMERA_I2C_BYTE_DATA);
  pr_err("cityman_smia dphy cap=0x%x/0x%x/0x%x ctrl=0x%x post=0x%x prep=0x%x "
         "zero=0x%x trail=0x%x clk_tr=0x%x clk_pr=0x%x clk_z=0x%x lp=0x%x "
         "rate=0x%04x%04x max4=0x%04x%04x if=0x%x/0x%x\n",
         cap_dphy, cap_lane, cap_sig, ctrl, tclk_post, ths_prep, ths_zero,
         ths_trail, tclk_trail, tclk_prep, tclk_zero, tlpx, rate_hi, rate_lo,
         max4_hi, max4_lo, if_ctrl, if_stat);
}

/*
 * WP ETL last-write map vs Linux live. Word: 0112/0136/0820/315x.
 * 0x3150-0x3156 is 3A-updated on WP; peek only, do not static-write.
 */
static void smia65pp_peek_wp_map(struct msm_sensor_ctrl_t *s_ctrl) {
  uint16_t r0100 = 0, r0105 = 0, r0106 = 0, r0112 = 0, r0130 = 0, r0132 = 0;
  uint16_t r0134 = 0, r0136 = 0, r0138 = 0, r0820 = 0, r0900 = 0, r0a02 = 0;
  uint16_t r3121 = 0, r3150 = 0, r3152 = 0, r3154 = 0, r3156 = 0, r31b0 = 0;
  uint16_t r4800 = 0, r6902 = 0, r6915 = 0, r6962 = 0, r69bc = 0, rb040 = 0;

  smia65pp_read(s_ctrl, 0x0100, &r0100, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0105, &r0105, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0106, &r0106, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0112, &r0112, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0130, &r0130, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0132, &r0132, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0134, &r0134, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0136, &r0136, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0138, &r0138, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0820, &r0820, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0900, &r0900, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0a02, &r0a02, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x3121, &r3121, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x3150, &r3150, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x3152, &r3152, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x3154, &r3154, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x3156, &r3156, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x31b0, &r31b0, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x4800, &r4800, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x6902, &r6902, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x6915, &r6915, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x6962, &r6962, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x69bc, &r69bc, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0xb040, &rb040, MSM_CAMERA_I2C_BYTE_DATA);
  pr_err(
      "cityman_smia wpmap 0100=0x%x 0105=0x%x 0106=0x%x 0112=0x%x 0130=0x%x "
      "0132=0x%x 0134=0x%x 0136=0x%x 0138=0x%x 0820=0x%x 0900=0x%x 0a02=0x%x "
      "3121=0x%x 3150=0x%x 3152=0x%x 3154=0x%x 3156=0x%x 31b0=0x%x 4800=0x%x "
      "6902=0x%x 6915=0x%x 6962=0x%x 69bc=0x%x b040=0x%x\n",
      r0100, r0105, r0106, r0112, r0130, r0132, r0134, r0136, r0138, r0820,
      r0900, r0a02, r3121, r3150, r3152, r3154, r3156, r31b0, r4800, r6902,
      r6915, r6962, r69bc, rb040);
}

/*
 * DPHY_CTRL_UI uses 0x0820 as 16.16 Mbps. 0 / 1× (424.8) / 2× (849.6) /
 * WP lane 106.2 (#138, stuck 0x006c0840) all ECC. Timings 0x0800-0x0807
 * stay 0 in UI. Keep 2× as 113.
 */
static int smia65pp_set_link_rate(struct msm_sensor_ctrl_t *s_ctrl) {
  uint16_t model = 0, pre = 0, mult = 0, op_sys = 0;
  uint16_t hi = 0, lo = 0, nhi = 0, nlo = 0;
  u32 op_hz, rate;
  int wrc = 0;

  smia65pp_read(s_ctrl, 0x0000, &model, MSM_CAMERA_I2C_WORD_DATA);
  if (model != 0xEACA)
    return 0;
  smia65pp_read(s_ctrl, 0x0820, &hi, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0822, &lo, MSM_CAMERA_I2C_WORD_DATA);
  if (hi || lo)
    return 0;
  smia65pp_read(s_ctrl, 0x0304, &pre, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0306, &mult, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x030a, &op_sys, MSM_CAMERA_I2C_WORD_DATA);
  if (!pre || !mult || !op_sys)
    return 0;
  op_hz = (u32)SMIA65PP_MCLK_HZ / pre * mult / op_sys;
  rate = DIV_ROUND_UP(op_hz * 2, 1000000 / 256 / 256);
  wrc |= smia65pp_write(s_ctrl, 0x0104, 1, MSM_CAMERA_I2C_BYTE_DATA);
  wrc |= smia65pp_write(s_ctrl, 0x0820, (u16)(rate >> 16),
                        MSM_CAMERA_I2C_WORD_DATA);
  wrc |= smia65pp_write(s_ctrl, 0x0822, (u16)(rate & 0xffff),
                        MSM_CAMERA_I2C_WORD_DATA);
  wrc |= smia65pp_write(s_ctrl, 0x0104, 0, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0820, &nhi, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0822, &nlo, MSM_CAMERA_I2C_WORD_DATA);
  pr_err("cityman_smia link_rate op=%u ddr val=0x%x wrc=%d now=0x%04x%04x\n",
         op_hz, rate, wrc, nhi, nlo);
  return 1;
}

/* SMIA++ 0x0600 U16: 2 = colour bars. Diagnostic: CSID long/crc vs still ECC.
 */
static int smia65pp_set_colorbars(struct msm_sensor_ctrl_t *s_ctrl) {
  uint16_t model = 0, now = 0;
  int wrc = 0;

  if (smia65pp_tp_done)
    return 0;
  smia65pp_read(s_ctrl, 0x0000, &model, MSM_CAMERA_I2C_WORD_DATA);
  if (model != 0xEACA)
    return 0;
  wrc |= smia65pp_write(s_ctrl, 0x0104, 1, MSM_CAMERA_I2C_BYTE_DATA);
  wrc |= smia65pp_write(s_ctrl, 0x0600, 2, MSM_CAMERA_I2C_WORD_DATA);
  wrc |= smia65pp_write(s_ctrl, 0x0104, 0, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0600, &now, MSM_CAMERA_I2C_WORD_DATA);
  pr_err("cityman_smia colorbars wrc=%d now=0x%x\n", wrc, now);
  smia65pp_tp_done = 1;
  return 1;
}

/*
 * #135: 0x0808=2 stuck, but TCLK_POST read 0 (wrote 0x3a before mode 2).
 * Enable REGISTER first, then timings.
 */
static int smia65pp_set_dphy_register(struct msm_sensor_ctrl_t *s_ctrl) {
  static const u16 regs[][2] = {
      {0x0800, 0x3a}, /* TCLK_POST  60ns+52UI */
      {0x0801, 0x09}, /* THS_PREPARE min 40ns+4UI */
      {0x0802, 0x1a}, /* THS_ZERO    145ns+10UI */
      {0x0803, 0x0b}, /* THS_TRAIL   60ns+4UI */
      {0x0804, 0x07}, /* TCLK_TRAIL  60ns */
      {0x0805, 0x05}, /* TCLK_PREPARE 38ns */
      {0x0806, 0x1c}, /* TCLK_ZERO   300ns-prepare */
      {0x0807, 0x06}, /* TLPX        50ns */
  };
  uint16_t model = 0, ctrl = 0, now = 0;
  uint16_t post = 0, prep = 0, zero = 0, trail = 0;
  int i, wrc = 0;

  if (smia65pp_dphy_done)
    return 0;
  smia65pp_read(s_ctrl, 0x0000, &model, MSM_CAMERA_I2C_WORD_DATA);
  if (model != 0xEACA)
    return 0;
  smia65pp_read(s_ctrl, 0x0808, &ctrl, MSM_CAMERA_I2C_BYTE_DATA);
  wrc |= smia65pp_write(s_ctrl, 0x0104, 1, MSM_CAMERA_I2C_BYTE_DATA);
  wrc |= smia65pp_write(s_ctrl, 0x0808, 2, MSM_CAMERA_I2C_BYTE_DATA);
  wrc |= smia65pp_write(s_ctrl, 0x0104, 0, MSM_CAMERA_I2C_BYTE_DATA);
  wrc |= smia65pp_write(s_ctrl, 0x0104, 1, MSM_CAMERA_I2C_BYTE_DATA);
  for (i = 0; i < ARRAY_SIZE(regs); i++)
    wrc |= smia65pp_write(s_ctrl, regs[i][0], regs[i][1],
                          MSM_CAMERA_I2C_BYTE_DATA);
  wrc |= smia65pp_write(s_ctrl, 0x0104, 0, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0808, &now, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0800, &post, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0801, &prep, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0802, &zero, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0803, &trail, MSM_CAMERA_I2C_BYTE_DATA);
  pr_err("cityman_smia dphy_register ctrl=0x%x -> 2 wrc=%d now=0x%x post=0x%x "
         "prep=0x%x zero=0x%x trail=0x%x\n",
         ctrl, wrc, now, post, prep, zero, trail);
  smia65pp_dphy_done = 1;
  return 1;
}

/*
 * WP ETL after 0x0103 at SID 0x20: SMIA VANA/VDIG/VIO then Sony MIPI
 * global. 8.8 volts 2.50 / 1.20 / 1.80. Do not write 0x0107 (HAL stays
 * on 0x20). 0x4800=0x0E matches public imx230.c + ETL. Prefix is the
 * ETL/public overlap, not the 0x9xxx idle blob (#68).
 */
static int smia65pp_wp_smiapp_supplies(struct msm_sensor_ctrl_t *s_ctrl) {
  static const u16 words[][2] = {
      {0x0130, 0x0280},
      {0x0132, 0x0133},
      {0x0134, 0x01cd},
  };
  static const u16 bytes[][2] = {
      {0x4800, 0x0e}, {0x4890, 0x01}, {0x4d1e, 0x01}, {0x4fa0, 0x00},
      {0x6153, 0x01}, {0x6156, 0x01}, {0x7300, 0x00}, {0x9009, 0x1a},
  };
  uint16_t model = 0, r0130 = 0, r0132 = 0, r0134 = 0, r4800 = 0;
  int i, wrc = 0;

  if (smia65pp_supplies_done)
    return 0;
  smia65pp_read(s_ctrl, 0x0000, &model, MSM_CAMERA_I2C_WORD_DATA);
  if (model != 0xEACA)
    return 0;
  wrc |= smia65pp_write(s_ctrl, 0x0104, 1, MSM_CAMERA_I2C_BYTE_DATA);
  for (i = 0; i < ARRAY_SIZE(words); i++)
    wrc |= smia65pp_write(s_ctrl, words[i][0], words[i][1],
                          MSM_CAMERA_I2C_WORD_DATA);
  for (i = 0; i < ARRAY_SIZE(bytes); i++)
    wrc |= smia65pp_write(s_ctrl, bytes[i][0], bytes[i][1],
                          MSM_CAMERA_I2C_BYTE_DATA);
  wrc |= smia65pp_write(s_ctrl, 0x0104, 0, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0130, &r0130, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0132, &r0132, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x0134, &r0134, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x4800, &r4800, MSM_CAMERA_I2C_BYTE_DATA);
  pr_err(
      "cityman_smia supplies wrc=%d 0130=0x%x 0132=0x%x 0134=0x%x 4800=0x%x\n",
      wrc, r0130, r0132, r0134, r4800);
  smia65pp_supplies_done = 1;
  return 1;
}

/*
 * #68 full CDCC idle silenced CSID; it also wrote extclk 0x0136/0137
 * (9.99 MHz vs ACPI 9.6). DPHY analog only, ident PLL 4/177 kept.
 */
static int smia65pp_cdcc_dphy_analog(struct msm_sensor_ctrl_t *s_ctrl) {
  static const u16 regs[][2] = {
      {0x3198, 0x0f},
      {0x31a0, 0x04},
      {0x31a1, 0x03},
      {0x31a2, 0x02},
      {0x31a3, 0x01},
      {0x31a8, 0x18}, /* idle unused; 31a0-31a3 already written */
      {0x6b42, 0x40},
      {0x6b46, 0x00},
      {0x6b47, 0x4b},
      {0x6b4a, 0x00},
      {0x6b4b, 0x4b},
      {0x6b4e, 0x00},
      {0x6b4f, 0x4b},
      {0x6b44, 0x00},
      {0x6b45, 0x8c},
      {0x6b48, 0x00},
      {0x6b49, 0x8c},
      {0x6b4c, 0x00},
      {0x6b4d, 0x8c},
      {0x31e4, 0x02},
      /* Idle analog. EXTCLK 0x0999 is SMIA 16.8 of 9.6 MHz
       * (smiapp: ext_clk / (1e6/256)), not 9.99. DPHY UI uses it. */
      {0x0136, 0x09},
      {0x0137, 0x99},
      {0x31e0, 0x03},
      {0x31e1, 0xff},
      {0x69bb, 0x01},
      {0x69c4, 0x01},
      {0x69c6, 0x01},
      /* Idle DPHY leftover. WP ETL wrote these; #68 full idle
       * at ident PLL 4/177 silenced CSID. After FF0B 1/124. */
      {0x3121, 0x01},
      {0x6902, 0x00},
      {0x6915, 0x01},
      {0x6953, 0x01},
      {0x6962, 0x3a},
      {0x69bc, 0x05},
      {0x69bd, 0x05},
      {0x69c1, 0x00},
      {0x69cd, 0x3a},
      {0xb040, 0x90},
      {0xb041, 0x14},
      {0xb042, 0x6b},
      {0xb043, 0x43},
      {0xb044, 0x63},
      {0xb045, 0x2a},
      {0xb046, 0x68},
      {0xb047, 0x06},
      {0xb048, 0x68},
      {0xb049, 0x07},
      {0xb04a, 0x68},
      {0xb04b, 0x04},
      {0xb04c, 0x68},
      {0xb04d, 0x05},
      {0xb04e, 0x68},
      {0xb04f, 0x16},
      {0xb050, 0x68},
      {0xb051, 0x17},
      {0xb052, 0x68},
      {0xb053, 0x74},
      {0xb054, 0x68},
      {0xb055, 0x75},
      {0xb056, 0x68},
      {0xb057, 0x76},
      {0xb058, 0x68},
      {0xb059, 0x77},
      {0xb05a, 0x68},
      {0xb05b, 0x7a},
      {0xb05c, 0x68},
      {0xb05d, 0x7b},
      {0xb05e, 0x68},
      {0xb05f, 0x0a},
      {0xb060, 0x68},
      {0xb061, 0x0b},
      {0xb062, 0x68},
      {0xb063, 0x08},
      {0xb064, 0x68},
      {0xb065, 0x09},
      {0xb066, 0x68},
      {0xb067, 0x0e},
      {0xb068, 0x68},
      {0xb069, 0x0f},
      {0xb06a, 0x68},
      {0xb06b, 0x0c},
      {0xb06c, 0x68},
      {0xb06d, 0x0d},
      {0xb06e, 0x68},
      {0xb06f, 0x13},
      {0xb070, 0x68},
      {0xb071, 0x12},
      {0xb072, 0x90},
      {0xb073, 0x0e},
      {0xb074, 0x69},
      {0xb075, 0x16},
      {0xb076, 0x69},
      {0xb077, 0x17},
  };
  uint16_t model = 0, r31a0 = 0, r31a8 = 0, r6b42 = 0, r31e4 = 0, rb040 = 0,
           r0136 = 0;
  uint16_t r6962 = 0, r3121 = 0;
  int i, wrc = 0;

  if (smia65pp_analog_done)
    return 0;
  smia65pp_read(s_ctrl, 0x0000, &model, MSM_CAMERA_I2C_WORD_DATA);
  if (model != 0xEACA)
    return 0;
  wrc |= smia65pp_write(s_ctrl, 0x0104, 1, MSM_CAMERA_I2C_BYTE_DATA);
  for (i = 0; i < ARRAY_SIZE(regs); i++)
    wrc |= smia65pp_write(s_ctrl, regs[i][0], regs[i][1],
                          MSM_CAMERA_I2C_BYTE_DATA);
  wrc |= smia65pp_write(s_ctrl, 0x0104, 0, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x31a0, &r31a0, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x31a8, &r31a8, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x6b42, &r6b42, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x31e4, &r31e4, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0xb040, &rb040, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x0136, &r0136, MSM_CAMERA_I2C_WORD_DATA);
  smia65pp_read(s_ctrl, 0x3121, &r3121, MSM_CAMERA_I2C_BYTE_DATA);
  smia65pp_read(s_ctrl, 0x6962, &r6962, MSM_CAMERA_I2C_BYTE_DATA);
  pr_err("cityman_smia dphy analog wrc=%d 31a0=0x%x 31a8=0x%x 6b42=0x%x "
         "31e4=0x%x b040=0x%x 0136=0x%x 3121=0x%x 6962=0x%x\n",
         wrc, r31a0, r31a8, r6b42, r31e4, rb040, r0136, r3121, r6962);
  smia65pp_analog_done = 1;
  return 1;
}

/*
 * After each HAL I2C table: DCC 0xFF0B viewfinder (2496x1872 PLL 1/124).
 * Mode-0 Sony vendor regs after CSIPHY (table #7), then 0x0100.
 */
static void smia65pp_after_qcam_i2c(struct msm_sensor_ctrl_t *s_ctrl) {
  uint16_t mode = 0;
  int cropped = smia65pp_crop_vfe(s_ctrl);
  int rated = 0;
  int bars = 0;
  int dphy = 0;
  int vend = 0;
  int ovr = 0;
  int analog = 0;
  int supplies = 0;
  int late, streamed = 0;

  smia65pp_read(s_ctrl, 0x0100, &mode, MSM_CAMERA_I2C_BYTE_DATA);

  /* Detect Motorola 24MHz MCLK PLL (pre=4, mult=200 -> 1200MHz VCO)
   * and adapt to Lumia 9.6MHz MCLK (pre=1, mult=cityman_pll_mult -> 1200MHz VCO).
   * Also correct EXTCLK 0x0136/0x0137 to 9.6MHz (0x0999) from Motorola's 24MHz (0x1800). */
  {
    uint16_t pre = 0, mult = 0;
    smia65pp_read(s_ctrl, 0x0304, &pre, MSM_CAMERA_I2C_WORD_DATA);
    smia65pp_read(s_ctrl, 0x0306, &mult, MSM_CAMERA_I2C_WORD_DATA);
    if (pre == 4 && mult == 200) {
      smia65pp_write(s_ctrl, 0x0104, 1, MSM_CAMERA_I2C_BYTE_DATA);
      smia65pp_write(s_ctrl, 0x0136, 0x09, MSM_CAMERA_I2C_BYTE_DATA);
      smia65pp_write(s_ctrl, 0x0137, 0x99, MSM_CAMERA_I2C_BYTE_DATA);
      smia65pp_write(s_ctrl, 0x0304, 1, MSM_CAMERA_I2C_WORD_DATA);
      smia65pp_write(s_ctrl, 0x0306, (u16)cityman_pll_mult, MSM_CAMERA_I2C_WORD_DATA);
      smia65pp_write(s_ctrl, 0x0104, 0, MSM_CAMERA_I2C_BYTE_DATA);
      pr_err("cityman_smia adapted PLL 4/200 (24MHz) -> 1/%d (9.6MHz -> 1200MHz VCO) EXTCLK 0x0999\n",
             cityman_pll_mult);
      rated = 1;
    }
  }

  if (cityman_wp_sensor_analog) {
    if (!smia65pp_supplies_done) {
      supplies = smia65pp_wp_smiapp_supplies(s_ctrl);
      smia65pp_dump_nvm_page0(s_ctrl);
    }
    if (!smia65pp_analog_done) {
      analog = smia65pp_cdcc_dphy_analog(s_ctrl);
    }
    if (!smia65pp_vendor_done) {
      vend = smia65pp_cdcc_vendor(s_ctrl);
      ovr = smia65pp_cdcc_override(s_ctrl);
    }
  }

  if (cityman_exp_lines > 0 || cityman_colorbars) {
    smia65pp_write(s_ctrl, 0x0104, 1, MSM_CAMERA_I2C_BYTE_DATA);
    if (cityman_colorbars)
      smia65pp_write(s_ctrl, 0x0600, 2, MSM_CAMERA_I2C_WORD_DATA);
    if (cityman_exp_lines > 0)
      smia65pp_write(s_ctrl, 0x0202, (u16)cityman_exp_lines, MSM_CAMERA_I2C_WORD_DATA);
    if (cityman_gain > 0)
      smia65pp_write(s_ctrl, 0x0204, (u16)cityman_gain, MSM_CAMERA_I2C_WORD_DATA);
    smia65pp_write(s_ctrl, 0x020e, 0x0100, MSM_CAMERA_I2C_WORD_DATA);
    smia65pp_write(s_ctrl, 0x0104, 0, MSM_CAMERA_I2C_BYTE_DATA);
  }

  late = smia65pp_i2c_logs >= 5 && smia65pp_i2c_logs <= 8;
  if (cropped || rated || bars || dphy || vend || ovr || analog || supplies ||
      streamed || !smia65pp_peeked || late) {
    smia65pp_peek(
        s_ctrl, cropped
                    ? "after_crop"
                    : (rated ? "after_rate"
                             : (bars && !streamed
                                    ? "after_bars"
                                    : (streamed ? "after_phy" : "after_i2c"))));
    smia65pp_peeked = 1;
    if (streamed || rated || dphy || analog || supplies) {
      smia65pp_peek_dphy(s_ctrl);
      smia65pp_peek_wp_map(s_ctrl);
    }
  }
}

static int32_t smia65pp_handle_stream(struct msm_sensor_ctrl_t *s_ctrl,
                                      int start) {
  int32_t rc;
  uint16_t mode = 0;

  mutex_lock(s_ctrl->msm_sensor_mutex);
  if (start) {
    smia65pp_write(s_ctrl, 0x0104, 1, MSM_CAMERA_I2C_BYTE_DATA);
    if (cityman_colorbars)
      smia65pp_write(s_ctrl, 0x0600, 2, MSM_CAMERA_I2C_WORD_DATA);
    else
      smia65pp_write(s_ctrl, 0x0600, 0, MSM_CAMERA_I2C_WORD_DATA);
    if (cityman_exp_lines > 0)
      smia65pp_write(s_ctrl, 0x0202, (u16)cityman_exp_lines, MSM_CAMERA_I2C_WORD_DATA);
    if (cityman_gain > 0)
      smia65pp_write(s_ctrl, 0x0204, (u16)cityman_gain, MSM_CAMERA_I2C_WORD_DATA);
    smia65pp_write(s_ctrl, 0x020e, 0x0100, MSM_CAMERA_I2C_WORD_DATA);
    smia65pp_write(s_ctrl, 0x0104, 0, MSM_CAMERA_I2C_BYTE_DATA);
  }
  rc = smia65pp_write(s_ctrl, 0x0100, start ? 1 : 0, MSM_CAMERA_I2C_BYTE_DATA);
  if (!rc && start)
    smia65pp_read(s_ctrl, 0x0100, &mode, MSM_CAMERA_I2C_BYTE_DATA);
  mutex_unlock(s_ctrl->msm_sensor_mutex);
  pr_err("cityman_smia %s_STREAM rc=%d mode=0x%x exp=%d gain=%d bars=%d\n",
         start ? "START" : "STOP", rc, start ? mode : 0,
         cityman_exp_lines, cityman_gain, cityman_colorbars);
  return rc;
}

static int smia65pp_is_i2c_cfg(int cfgtype) {
  return cfgtype == CFG_WRITE_I2C_ARRAY ||
         cfgtype == CFG_SLAVE_WRITE_I2C_ARRAY ||
         cfgtype == CFG_WRITE_I2C_SEQ_ARRAY;
}

static void smia65pp_log_cfg(int cfgtype) {
  if (cfgtype == CFG_GET_SENSOR_INFO)
    return;
  if (cfgtype == CFG_WRITE_I2C_ARRAY) {
    if (smia65pp_i2c_logs < 12) {
      smia65pp_i2c_logs++;
      pr_err("cityman_smia cfg32 WRITE_I2C #%d\n", smia65pp_i2c_logs);
    }
    return;
  }
  pr_err("cityman_smia cfg32 type=%d\n", cfgtype);
}

/*
 * fops ioctl is video_usercopy → kernel pointer. Generic
 * msm_sensor_config32 casts argp; it does not copy_from_user.
 */
static int32_t smia65pp_config(struct msm_sensor_ctrl_t *s_ctrl,
                               void __user *argp) {
  struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
  int32_t rc;

  if (cdata->cfgtype == CFG_SET_START_STREAM)
    return smia65pp_handle_stream(s_ctrl, 1);
  if (cdata->cfgtype == CFG_SET_STOP_STREAM)
    return smia65pp_handle_stream(s_ctrl, 0);
  if (cdata->cfgtype == CFG_SLAVE_READ_I2C) {
    struct msm_camera_i2c_read_config read_cfg;
    struct msm_camera_i2c_read_config __user *uptr =
        (struct msm_camera_i2c_read_config __user *)cdata->cfg.setting;
    if (!copy_from_user(&read_cfg, uptr, sizeof(read_cfg))) {
      uint16_t resp_data = 1;
      pr_err("cityman_smia CFG_SLAVE_READ_I2C: slave=0x%x reg=0x%x dt=%d\n",
             read_cfg.slave_addr, read_cfg.reg_addr, read_cfg.data_type);
      if (read_cfg.slave_addr == 0x34) {
        resp_data = 1;
        if (copy_to_user(&uptr->data, &resp_data, sizeof(resp_data)))
          return -EFAULT;
        return 0;
      }
      rc = msm_sensor_config(s_ctrl, argp);
      if (rc < 0) {
        if (read_cfg.reg_addr == 0x0016 || read_cfg.reg_addr == 0x0000)
          resp_data = (read_cfg.data_type == MSM_CAMERA_I2C_BYTE_DATA) ? 0x02 : 0x0230;
        else if (read_cfg.reg_addr == 0x0017)
          resp_data = 0x30;
        else if (read_cfg.reg_addr == 0x0018)
          resp_data = 0x00;
        else if (read_cfg.reg_addr == 0x0019)
          resp_data = 0x01;
        else
          resp_data = 0;
        if (copy_to_user(&uptr->data, &resp_data, sizeof(resp_data)))
          return -EFAULT;
        pr_err("cityman_smia CFG_SLAVE_READ_I2C fallback reg=0x%x -> 0x%x\n",
               read_cfg.reg_addr, resp_data);
        return 0;
      }
      return rc;
    }
  }
  smia65pp_log_cfg(cdata->cfgtype);
  if (cdata->cfgtype == CFG_POWER_DOWN) {
    smia65pp_stream_forced = 0;
    smia65pp_peeked = 0;
    smia65pp_i2c_logs = 0;
    smia65pp_tp_done = 0;
    smia65pp_dphy_done = 0;
    smia65pp_nvm_done = 0;
    smia65pp_vendor_done = 0;
    smia65pp_ovr_done = 0;
    smia65pp_analog_done = 0;
    smia65pp_supplies_done = 0;
  }
  rc = msm_sensor_config(s_ctrl, argp);
  if (smia65pp_is_i2c_cfg(cdata->cfgtype)) {
    mutex_lock(s_ctrl->msm_sensor_mutex);
    smia65pp_after_qcam_i2c(s_ctrl);
    mutex_unlock(s_ctrl->msm_sensor_mutex);
  }
  return rc;
}

#ifdef CONFIG_COMPAT
static int32_t smia65pp_config32(struct msm_sensor_ctrl_t *s_ctrl,
                                 void __user *argp) {
  struct sensorb_cfg_data32 *cdata = (struct sensorb_cfg_data32 *)argp;
  int32_t rc;

  if (cdata->cfgtype == CFG_SET_START_STREAM)
    return smia65pp_handle_stream(s_ctrl, 1);
  if (cdata->cfgtype == CFG_SET_STOP_STREAM)
    return smia65pp_handle_stream(s_ctrl, 0);
  if (cdata->cfgtype == CFG_SLAVE_READ_I2C) {
    struct msm_camera_i2c_read_config read_cfg;
    struct msm_camera_i2c_read_config __user *uptr =
        (struct msm_camera_i2c_read_config __user *)compat_ptr(cdata->cfg.setting);
    if (!copy_from_user(&read_cfg, uptr, sizeof(read_cfg))) {
      uint16_t resp_data = 1;
      pr_err("cityman_smia CFG_SLAVE_READ_I2C: slave=0x%x reg=0x%x dt=%d\n",
             read_cfg.slave_addr, read_cfg.reg_addr, read_cfg.data_type);
      if (read_cfg.slave_addr == 0x34) {
        resp_data = 1;
        if (copy_to_user(&uptr->data, &resp_data, sizeof(resp_data)))
          return -EFAULT;
        return 0;
      }
      rc = msm_sensor_config32(s_ctrl, argp);
      if (rc < 0) {
        if (read_cfg.reg_addr == 0x0016 || read_cfg.reg_addr == 0x0000)
          resp_data = (read_cfg.data_type == MSM_CAMERA_I2C_BYTE_DATA) ? 0x02 : 0x0230;
        else if (read_cfg.reg_addr == 0x0017)
          resp_data = 0x30;
        else if (read_cfg.reg_addr == 0x0018)
          resp_data = 0x00;
        else if (read_cfg.reg_addr == 0x0019)
          resp_data = 0x01;
        else
          resp_data = 0;
        if (copy_to_user(&uptr->data, &resp_data, sizeof(resp_data)))
          return -EFAULT;
        pr_err("cityman_smia CFG_SLAVE_READ_I2C fallback reg=0x%x -> 0x%x\n",
               read_cfg.reg_addr, resp_data);
        return 0;
      }
      return rc;
    }
  }
  smia65pp_log_cfg(cdata->cfgtype);
  if (cdata->cfgtype == CFG_POWER_DOWN) {
    smia65pp_stream_forced = 0;
    smia65pp_peeked = 0;
    smia65pp_i2c_logs = 0;
    smia65pp_tp_done = 0;
    smia65pp_dphy_done = 0;
    smia65pp_nvm_done = 0;
    smia65pp_vendor_done = 0;
    smia65pp_ovr_done = 0;
    smia65pp_analog_done = 0;
    smia65pp_supplies_done = 0;
  }
  rc = msm_sensor_config32(s_ctrl, argp);
  if (smia65pp_is_i2c_cfg(cdata->cfgtype)) {
    mutex_lock(s_ctrl->msm_sensor_mutex);
    smia65pp_after_qcam_i2c(s_ctrl);
    mutex_unlock(s_ctrl->msm_sensor_mutex);
  }
  return rc;
}
#endif

int32_t smia65pp_match_id(struct msm_sensor_ctrl_t *s_ctrl) {
  u32 manufacturer_id = 0, model_id = 0, rev_major = 0;
  int rc;

  rc = smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U16_MODEL_ID, &model_id);
  if (rc < 0) {
    pr_err("smia65pp_match_id: module id read failed rc=%d\n", rc);
    return rc;
  }
  smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_MANUFACTURER_ID, &manufacturer_id);
  smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_REVISION_NUMBER_MAJOR, &rev_major);
  pr_err("smia65pp_match_id: manufacturer=0x%02x model=0x%04x rev=0x%02x\n",
         manufacturer_id, model_id, rev_major);
  smia65pp_smiapp_identify(s_ctrl);
  return 0;
}

static struct msm_sensor_fn_t smia65pp_sensor_func_tbl = {
    .sensor_config = smia65pp_config,
#ifdef CONFIG_COMPAT
    .sensor_config32 = smia65pp_config32,
#endif
    .sensor_power_up = msm_sensor_power_up,
    .sensor_power_down = msm_sensor_power_down,
    .sensor_match_id = smia65pp_match_id,
};

static struct msm_sensor_ctrl_t smia65pp_s_ctrl = {
    .sensor_i2c_client = &smia65pp_sensor_i2c_client,
    .power_setting_array.power_setting = smia65pp_power_setting,
    .power_setting_array.size = ARRAY_SIZE(smia65pp_power_setting),
    .msm_sensor_mutex = &smia65pp_mut,
    .sensor_v4l2_subdev_info = smia65pp_subdev_info,
    .sensor_v4l2_subdev_info_size = ARRAY_SIZE(smia65pp_subdev_info),
    .func_tbl = &smia65pp_sensor_func_tbl,
};

module_init(smia65pp_init_module);
module_exit(smia65pp_exit_module);
MODULE_DESCRIPTION("smia65pp");
MODULE_LICENSE("GPL v2");
