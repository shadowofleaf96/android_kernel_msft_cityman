/* Copyright (c) 2011-2014, 2017 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "msm_csiphy.h"
#include "include/msm_csiphy_2_0_hwreg.h"
#include "include/msm_csiphy_2_2_hwreg.h"
#include "include/msm_csiphy_3_0_hwreg.h"
#include "include/msm_csiphy_3_1_hwreg.h"
#include "include/msm_csiphy_3_2_hwreg.h"
#include "msm_camera_io_util.h"
#include "msm_sd.h"
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/irqreturn.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/workqueue.h>

#define DBG_CSIPHY 0

/* #49: 0x0234 == 0x4320 (pkts+ECC). Try longer HS-settle. HAL 0x1b is 135ns
 * @ 200 MHz; 0x28 is 200ns. echo -1 leaves the HAL value. */
static int cityman_settle = 0x23;
module_param_named(settle, cityman_settle, int, 0644);
MODULE_PARM_DESC(settle, "CSIPHY settle_cnt; -1 = leave HAL value");

/* 0 = CAF defaults. 1 = WP live Hill overrides: CFG3=0x16, CFG4=0xff, CFG5=0x22, CFG2=0x3f. */
static int cityman_wp_analog = 0;
module_param_named(wp_analog, cityman_wp_analog, int, 0644);
MODULE_PARM_DESC(wp_analog, "CSIPHY 20nm WP analog overrides (CFG3=0x16 CFG4=0xff CFG5=0x22)");

/* 20nm lnn_misc1 lane-id uses 0x4 (clk) / 0x8|n (data). Bit 0 is unused.
 * OR 1 on every enabled lane = try P/N invert. echo 0 to disable. */
static int cityman_pn_invert = 0;
module_param_named(pn_invert, cityman_pn_invert, int, 0644);

static int cityman_clk_mux = -1;
module_param_named(clk_mux, cityman_clk_mux, int, 0644);
MODULE_PARM_DESC(pn_invert, "OR 1 into CSIPHY 20nm lnn_misc1 (P/N invert)");

/* Lumia 950 XL: 4 data lanes on Pins 0, 2, 3, 4 and Clock on Pin 1.
 * Mask 0x1F = pins 0, 1, 2, 3, 4. */
static int cityman_lane_cnt = 4;
module_param_named(lane_cnt, cityman_lane_cnt, int, 0644);
MODULE_PARM_DESC(lane_cnt, "CSIPHY lane_cnt override; -1 = DT/HAL");

static int cityman_lane_mask = 0x1f;
module_param_named(lane_mask, cityman_lane_mask, int, 0644);
MODULE_PARM_DESC(lane_mask, "CSIPHY lane_mask override; -1 = DT/HAL");

static int cityman_clk_lane = 1;
module_param_named(clk_lane, cityman_clk_lane, int, 0644);
MODULE_PARM_DESC(clk_lane, "CSIPHY physical clock lane index");

static struct csiphy_device *cityman_late_csiphy;
static void cityman_csiphy_late_fn(struct work_struct *w);
static DECLARE_DELAYED_WORK(cityman_csiphy_late, cityman_csiphy_late_fn);

static void cityman_csiphy_late_fn(struct work_struct *w) {
  struct csiphy_device *d = cityman_late_csiphy;
  void __iomem *base;
  int j;

  if (!d || !d->base)
    return;
  base = d->base;
  pr_err("cityman_csiphy late irq 0=0x%x 1=0x%x 2=0x%x 3=0x%x 4=0x%x "
         "mask0=0x%x pwr=0x%x\n",
         msm_camera_io_r(base + 0x18c), msm_camera_io_r(base + 0x190),
         msm_camera_io_r(base + 0x194), msm_camera_io_r(base + 0x198),
         msm_camera_io_r(base + 0x19c), msm_camera_io_r(base + 0x1ac),
         msm_camera_io_r(
             base + d->ctrl_reg->csiphy_reg.mipi_csiphy_glbl_pwr_cfg_addr));
  for (j = 0; j < 5; j++) {
    void __iomem *ln = base + 0x40 * j;

    pr_err("cityman_csiphy late j=%d 00=0x%x 04=0x%x 08=0x%x 0c=0x%x 10=0x%x "
           "20=0x%x 28=0x%x\n",
           j, msm_camera_io_r(ln + 0x00), msm_camera_io_r(ln + 0x04),
           msm_camera_io_r(ln + 0x08), msm_camera_io_r(ln + 0x0c),
           msm_camera_io_r(ln + 0x10), msm_camera_io_r(ln + 0x20),
           msm_camera_io_r(ln + 0x28));
  }
}

/* #63 half CSI same EOT+ECC. cfg4 reset is 0x5; clear bit 0. */
static int cityman_cfg4_clr0 = 0;
module_param_named(cfg4_clr0, cityman_cfg4_clr0, int, 0644);
MODULE_PARM_DESC(cfg4_clr0, "Clear CSIPHY LNn_CFG4 bit 0");

#define V4L2_IDENT_CSIPHY 50003
#define CSIPHY_VERSION_V22 0x01
#define CSIPHY_VERSION_V20 0x00
#define CSIPHY_VERSION_V30 0x10
#define CSIPHY_VERSION_V31 0x31
#define CSIPHY_VERSION_V32 0x32
#define MSM_CSIPHY_DRV_NAME "msm_csiphy"
#define CLK_LANE_OFFSET 1
#define NUM_LANES_OFFSET 4

static struct camera_vreg_t csiphy_vreg_info[] = {
    {"qcom,mipi-csi-vdd", 0, 0, 12000},
};

#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

static struct msm_cam_clk_info csiphy_clk_info[CSIPHY_NUM_CLK_MAX];
static struct v4l2_file_operations msm_csiphy_v4l2_subdev_fops;

static int
msm_csiphy_lane_config(struct csiphy_device *csiphy_dev,
                       struct msm_camera_csiphy_params *csiphy_params) {
  int rc = 0;
  int j = 0, curr_lane = 0;
  uint32_t val = 0, clk_rate = 0, round_rate = 0;
  uint8_t lane_cnt = 0;
  uint16_t lane_mask = 0;
  void __iomem *csiphybase;
  uint8_t csiphy_id = csiphy_dev->pdev->id;
  int32_t lane_val = 0, lane_right = 0, num_lanes = 0;
  struct clk **csid_phy_clk_ptr;
  int ratio = 1;

  csiphybase = csiphy_dev->base;
  if (!csiphybase) {
    pr_err("%s: csiphybase NULL\n", __func__);
    return -EINVAL;
  }

  if (cityman_lane_mask >= 0) {
    csiphy_dev->lane_mask[csiphy_id] = cityman_lane_mask;
    csiphy_params->lane_mask = cityman_lane_mask;
  }
  if (cityman_lane_cnt > 0) {
    csiphy_params->lane_cnt = cityman_lane_cnt;
  }

  csiphy_dev->lane_mask[csiphy_id] |= csiphy_params->lane_mask;
  lane_mask = csiphy_dev->lane_mask[csiphy_id];
  lane_cnt = csiphy_params->lane_cnt;
  if (csiphy_params->lane_cnt < 1 || csiphy_params->lane_cnt > 4) {
    pr_err("%s: unsupported lane cnt %d\n", __func__, csiphy_params->lane_cnt);
    return rc;
  }

  csid_phy_clk_ptr = csiphy_dev->csiphy_clk;
  if (!csid_phy_clk_ptr) {
    pr_err("csiphy_timer_src_clk get failed\n");
    return -EINVAL;
  }

  clk_rate = (csiphy_params->csiphy_clk > 0) ? csiphy_params->csiphy_clk
                                             : csiphy_dev->csiphy_max_clk;
  /* smia7 advertises 53.1 MHz vt. That is below CSIPHY timer min. */
  if (clk_rate < 100000000)
    clk_rate = csiphy_dev->csiphy_max_clk;
  round_rate =
      clk_round_rate(csid_phy_clk_ptr[csiphy_dev->csiphy_clk_index], clk_rate);
  if (round_rate >= csiphy_dev->csiphy_max_clk)
    round_rate = csiphy_dev->csiphy_max_clk;
  else if (round_rate) {
    ratio = csiphy_dev->csiphy_max_clk / round_rate;
    if (ratio > 1)
      csiphy_params->settle_cnt = csiphy_params->settle_cnt / ratio;
  }
  if (csiphy_params->settle_cnt < 8)
    csiphy_params->settle_cnt = 0x0E;
  if (cityman_settle >= 0)
    csiphy_params->settle_cnt = cityman_settle;

  pr_err("cityman_csiphy id=%d lanes=%u mask=0x%x settle=0x%x combo=%u csid=%u "
         "usr_clk=%u use=%u round=%u hw=0x%x nm20=%d\n",
         csiphy_id, csiphy_params->lane_cnt, csiphy_params->lane_mask,
         csiphy_params->settle_cnt, csiphy_params->combo_mode,
         csiphy_params->csid_core, csiphy_params->csiphy_clk, clk_rate,
         round_rate, csiphy_dev->hw_version, csiphy_dev->is_3_1_20nm_hw);
  rc = clk_set_rate(csid_phy_clk_ptr[csiphy_dev->csiphy_clk_index], round_rate);
  if (rc < 0) {
    pr_err("csiphy_timer_src_clk set failed\n");
    return rc;
  }
  {
    void __iomem *cgc = ioremap(0xFDA00000, 0x50);
    void __iomem *mmcc = ioremap(0xFD8C3000, 0x800);
    void __iomem *tcsr = ioremap(0xFD512028, 4);

    if (cgc) {
      pr_err("cityman_cgc before 00=%x 10=%x 20=%x 24=%x 28=%x 2c=%x 30=%x "
             "34=%x 38=%x 3c=%x 40=%x 44=%x\n",
             msm_camera_io_r(cgc), msm_camera_io_r(cgc + 0x10),
             msm_camera_io_r(cgc + 0x20), msm_camera_io_r(cgc + 0x24),
             msm_camera_io_r(cgc + 0x28), msm_camera_io_r(cgc + 0x2c),
             msm_camera_io_r(cgc + 0x30), msm_camera_io_r(cgc + 0x34),
             msm_camera_io_r(cgc + 0x38), msm_camera_io_r(cgc + 0x3c),
             msm_camera_io_r(cgc + 0x40), msm_camera_io_r(cgc + 0x44));
      /* WP VF enables. Mux stays 0. */
      msm_camera_io_w(0xf, cgc + 0x24);
      msm_camera_io_w(0x3f, cgc + 0x2c);
      msm_camera_io_w(3, cgc + 0x34);
      msm_camera_io_w(3, cgc + 0x3c);
      msm_camera_io_w(3, cgc + 0x44);
      mb();
      pr_err("cityman_cgc after 24=%x 2c=%x 34=%x 3c=%x 44=%x\n",
             msm_camera_io_r(cgc + 0x24), msm_camera_io_r(cgc + 0x2c),
             msm_camera_io_r(cgc + 0x34), msm_camera_io_r(cgc + 0x3c),
             msm_camera_io_r(cgc + 0x44));
      iounmap(cgc);
    } else
      pr_err("cityman_cgc ioremap fail\n");
    if (mmcc) {
      pr_err("cityman_mmcc phy=%x csi0=%x csi0_cbcr=%x csi_vfe0=%x vfe0=%x "
             "mclk0=%x\n",
             msm_camera_io_r(mmcc + 0x4), msm_camera_io_r(mmcc + 0x94),
             msm_camera_io_r(mmcc + 0xb4), msm_camera_io_r(mmcc + 0x704),
             msm_camera_io_r(mmcc + 0x604), msm_camera_io_r(mmcc + 0x364));
      iounmap(mmcc);
    }
    if (tcsr) {
      pr_err("cityman_tcsr fd512028=%x (WP 009690e1)\n", msm_camera_io_r(tcsr));
      iounmap(tcsr);
    }
  }

  CDBG("%s csiphy_params, mask = 0x%x cnt = %d\n", __func__,
       csiphy_params->lane_mask, csiphy_params->lane_cnt);
  CDBG("%s csiphy_params, settle cnt = 0x%x csid %d\n", __func__,
       csiphy_params->settle_cnt, csiphy_params->csid_core);

  if (csiphy_dev->hw_version >= CSIPHY_VERSION_V30) {
    val = msm_camera_io_r(csiphy_dev->clk_mux_base);
    if (csiphy_params->combo_mode &&
        (csiphy_params->lane_mask & 0x18) == 0x18) {
      val &= ~0xf0;
      val |= csiphy_params->csid_core << 4;
      val &= ~0xf;
      val |= csiphy_params->csid_core;
    } else {
      val &= ~0xf;
      val |= csiphy_params->csid_core;
    }
    if (cityman_clk_mux != -1) {
      val &= ~0xf;
      val |= (cityman_clk_mux & 0xf);
    }
    msm_camera_io_w(val, csiphy_dev->clk_mux_base);
    CDBG("%s clk mux addr %pK val 0x%x\n", __func__, csiphy_dev->clk_mux_base,
         val);
    mb();
  }
  msm_camera_io_w(
      0x1,
      csiphybase +
          csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_glbl_t_init_cfg0_addr);
  msm_camera_io_w(
      0x1, csiphybase +
               csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_t_wakeup_cfg0_addr);

  if (csiphy_dev->hw_version < CSIPHY_VERSION_V30) {
    val = 0x3;
    msm_camera_io_w(
        (lane_mask << 2) | val,
        csiphybase +
            csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_glbl_pwr_cfg_addr);
    msm_camera_io_w(
        0x10, csiphybase +
                  csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnck_cfg2_addr);
    msm_camera_io_w(
        csiphy_params->settle_cnt,
        csiphybase +
            csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnck_cfg3_addr);
    msm_camera_io_w(
        0x24,
        csiphybase +
            csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_interrupt_mask0_addr);
    msm_camera_io_w(
        0x24,
        csiphybase +
            csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_interrupt_clear0_addr);
  } else {
    val = 0x1;
    msm_camera_io_w(
        (lane_mask << 1) | val,
        csiphybase +
            csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_glbl_pwr_cfg_addr);
    msm_camera_io_w(
        csiphy_params->combo_mode
            << csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_mode_config_shift,
        csiphybase +
            csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_glbl_reset_addr);
  }

  lane_mask &= 0x1f;
  while (lane_mask & 0x1f) {
    if (!(lane_mask & 0x1)) {
      j++;
      lane_mask >>= 1;
      continue;
    }
    msm_camera_io_w(
        0x10, csiphybase +
                  csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg2_addr +
                  0x40 * j);
    msm_camera_io_w(
        csiphy_params->settle_cnt,
        csiphybase +
            csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg3_addr +
            0x40 * j);
    if (cityman_wp_analog) {
      /* WP live Hill CFG3=0x16 (Linux settle 0x23). */
      /* Commenting out to allow the HAL's settle_cnt to take effect
      msm_camera_io_w(
          0x16, csiphybase +
                    csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg3_addr +
                    0x40 * j);
      */
      pr_err("cityman_csiphy wp cfg3 j=%d settle=0x%x now=0x%x\n", j,
             csiphy_params->settle_cnt,
             msm_camera_io_r(
                 csiphybase +
                 csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg3_addr +
                 0x40 * j));
    }
    msm_camera_io_w(
        csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_interrupt_mask_val,
        csiphybase +
            csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_interrupt_mask_addr +
            0x4 * j);
    msm_camera_io_w(
        csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_interrupt_mask_val,
        csiphybase +
            csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_interrupt_clear_addr +
            0x4 * j);
    if (csiphy_dev->is_3_1_20nm_hw == 1) {
      if (j > CLK_LANE_OFFSET) {
        lane_right = 0x8;
        num_lanes = (lane_cnt - curr_lane) << NUM_LANES_OFFSET;
        if (lane_cnt < curr_lane) {
          pr_err("%s: Lane_cnt is less than curr_lane number\n", __func__);
          return -EINVAL;
        }
        lane_val = lane_right | num_lanes;
      } else if (j == 1) {
        lane_val = 0x4;
      } else {
        lane_val = 0x0;
      }
      if (csiphy_params->combo_mode == 1) {
        /*
         * In the case of combo mode, the clock is always
         * 4th lane for the second sensor.
         * So check whether the sensor is of one lane
         * sensor and curr_lane for 0.
         */
        if (curr_lane == 0 && ((csiphy_params->lane_mask & 0x18) == 0x18))
          lane_val = 0x4;
      }
      /* 0=off 1=all (same as off on a differential link)
       * 2=clock only (misc1 0x4) 3=data only (misc1 bit3). */
      if (cityman_pn_invert == 1 ||
          (cityman_pn_invert == 2 && lane_val == 0x4) ||
          (cityman_pn_invert == 3 && (lane_val & 0x8)))
        lane_val |= 0x1;
      pr_err("cityman_csiphy ln j=%d misc1=0x%x pn=%d\n", j, lane_val,
             cityman_pn_invert);
      msm_camera_io_w(
          lane_val,
          csiphybase +
              csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_misc1_addr +
              0x40 * j);
      msm_camera_io_w(
          0x17, csiphybase +
                    csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_test_imp +
                    0x40 * j);
      if (cityman_cfg4_clr0) {
        uint32_t c4 = msm_camera_io_r(
            csiphybase +
            csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg4_addr +
            0x40 * j);

        msm_camera_io_w(
            0, csiphybase +
                   csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg4_addr +
                   0x40 * j);
      }
      if (cityman_wp_analog) {
        uint32_t c4 = msm_camera_io_r(
            csiphybase +
            csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg4_addr +
            0x40 * j);
        uint32_t c5 = msm_camera_io_r(
            csiphybase +
            csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg5_addr +
            0x40 * j);

        /* WP live Hill: CFG4=0xff CFG5=0x22. */
        msm_camera_io_w(
            0xff,
            csiphybase +
                csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg4_addr +
                0x40 * j);
        msm_camera_io_w(
            0x22,
            csiphybase +
                csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg5_addr +
                0x40 * j);
        {
          uint32_t c2 = msm_camera_io_r(
              csiphybase +
              csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg2_addr +
              0x40 * j);

          msm_camera_io_w(
              0x3f,
              csiphybase +
                  csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg2_addr +
                  0x40 * j);
          pr_err(
              "cityman_csiphy wp cfg2 j=%d was=0x%x now=0x%x\n", j, c2,
              msm_camera_io_r(
                  csiphybase +
                  csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg2_addr +
                  0x40 * j));
        }
        pr_err("cityman_csiphy wp analog j=%d cfg4 was=0x%x now=0x%x cfg5 "
               "was=0x%x now=0x%x\n",
               j, c4,
               msm_camera_io_r(
                   csiphybase +
                   csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg4_addr +
                   0x40 * j),
               c5,
               msm_camera_io_r(
                   csiphybase +
                   csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg5_addr +
                   0x40 * j));
      }
      curr_lane++;
    }
    j++;
    lane_mask >>= 1;
  }
  /* 20nm lane block is 0x40, not newer 2PH 0x200+CTRL9. Dump analog. */
  for (j = 0; j < 5; j++) {
    void __iomem *ln = csiphybase + 0x40 * j;

    pr_err("cityman_csiphy dump j=%d 00=0x%x 04=0x%x 08=0x%x 0c=0x%x 10=0x%x "
           "14=0x%x 18=0x%x 1c=0x%x 20=0x%x 28=0x%x 2c=0x%x\n",
           j, msm_camera_io_r(ln + 0x00), msm_camera_io_r(ln + 0x04),
           msm_camera_io_r(ln + 0x08), msm_camera_io_r(ln + 0x0c),
           msm_camera_io_r(ln + 0x10), msm_camera_io_r(ln + 0x14),
           msm_camera_io_r(ln + 0x18), msm_camera_io_r(ln + 0x1c),
           msm_camera_io_r(ln + 0x20), msm_camera_io_r(ln + 0x28),
           msm_camera_io_r(ln + 0x2c));
  }
  pr_err(
      "cityman_csiphy irq 0=0x%x 1=0x%x 2=0x%x 3=0x%x 4=0x%x pwr=0x%x\n",
      msm_camera_io_r(csiphybase + 0x18c), msm_camera_io_r(csiphybase + 0x190),
      msm_camera_io_r(csiphybase + 0x194), msm_camera_io_r(csiphybase + 0x198),
      msm_camera_io_r(csiphybase + 0x19c),
      msm_camera_io_r(
          csiphybase +
          csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_glbl_pwr_cfg_addr));
  cityman_late_csiphy = csiphy_dev;
  schedule_delayed_work(&cityman_csiphy_late, msecs_to_jiffies(300));
  return rc;
}

static irqreturn_t msm_csiphy_irq(int irq_num, void *data) {
  uint32_t irq;
  int i;
  struct csiphy_device *csiphy_dev = data;

  for (i = 0; i < 8; i++) {
    irq = msm_camera_io_r(
        csiphy_dev->base +
        csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_interrupt_status0_addr +
        0x4 * i);
    msm_camera_io_w(
        irq,
        csiphy_dev->base +
            csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_interrupt_clear0_addr +
            0x4 * i);
    pr_err("%s MIPI_CSIPHY%d_INTERRUPT_STATUS%d = 0x%x\n", __func__,
           csiphy_dev->pdev->id, i, irq);
    msm_camera_io_w(
        0x1,
        csiphy_dev->base +
            csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_glbl_irq_cmd_addr);
    msm_camera_io_w(
        0x0,
        csiphy_dev->base +
            csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_glbl_irq_cmd_addr);
    msm_camera_io_w(
        0x0,
        csiphy_dev->base +
            csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_interrupt_clear0_addr +
            0x4 * i);
  }
  return IRQ_HANDLED;
}

static void msm_csiphy_reset(struct csiphy_device *csiphy_dev) {
  msm_camera_io_w(
      0x1, csiphy_dev->base +
               csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_glbl_reset_addr);
  usleep_range(5000, 8000);
  msm_camera_io_w(
      0x0, csiphy_dev->base +
               csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_glbl_reset_addr);
}

static int msm_csiphy_subdev_g_chip_ident(struct v4l2_subdev *sd,
                                          struct v4l2_dbg_chip_ident *chip) {
  BUG_ON(!chip);
  chip->ident = V4L2_IDENT_CSIPHY;
  chip->revision = 0;
  return 0;
}

#if DBG_CSIPHY
static int msm_csiphy_init(struct csiphy_device *csiphy_dev) {
  int rc = 0;
  if (csiphy_dev == NULL) {
    pr_err("%s: csiphy_dev NULL\n", __func__);
    rc = -ENOMEM;
    return rc;
  }

  CDBG("%s:%d called\n", __func__, __LINE__);
  if (csiphy_dev->csiphy_state == CSIPHY_POWER_UP) {
    pr_err("%s: csiphy invalid state %d\n", __func__, csiphy_dev->csiphy_state);
    rc = -EINVAL;
    return rc;
  }
  CDBG("%s:%d called\n", __func__, __LINE__);

  if (csiphy_dev->ref_count++) {
    CDBG("%s csiphy refcount = %d\n", __func__, csiphy_dev->ref_count);
    return rc;
  }
  CDBG("%s:%d called\n", __func__, __LINE__);

  csiphy_dev->base =
      ioremap(csiphy_dev->mem->start, resource_size(csiphy_dev->mem));
  if (!csiphy_dev->base) {
    pr_err("%s: csiphy_dev->base NULL\n", __func__);
    csiphy_dev->ref_count--;
    rc = -ENOMEM;
    return rc;
  }
  CDBG("%s:%d called\n", __func__, __LINE__);

  rc = msm_camera_config_vreg(&csiphy_dev->pdev->dev, csiphy_vreg_info,
                              ARRAY_SIZE(csiphy_vreg_info), NULL, 0,
                              &csiphy_dev->csi_vdd, 1);
  if (rc < 0) {
    pr_err("%s: regulator config failed\n", __func__);
    goto csiphy_vreg_config_fail;
  }
  rc = msm_camera_enable_vreg(&csiphy_dev->pdev->dev, csiphy_vreg_info,
                              ARRAY_SIZE(csiphy_vreg_info), NULL, 0,
                              &csiphy_dev->csi_vdd, 1);
  if (rc < 0) {
    pr_err("%s: regulator enable failed\n", __func__);
    goto csiphy_vreg_enable_fail;
  }

  csiphy_dev->reg_ptr =
      regulator_get(&(csiphy_dev->pdev->dev), "qcom,gdscr-vdd");
  if (IS_ERR_OR_NULL(csiphy_dev->reg_ptr)) {
    pr_err(" %s: Failed in getting TOP gdscr regulator handle", __func__);
  } else {
    rc = regulator_enable(csiphy_dev->reg_ptr);
    if (rc) {
      pr_err(" %s: regulator enable failed for GDSCR\n", __func__);
      goto csiphy_enable_regulator_gdscr_fail;
    }
  }

  if (csiphy_dev->hw_dts_version < CSIPHY_VERSION_V30) {
    rc = msm_cam_clk_enable(&csiphy_dev->pdev->dev, csiphy_clk_info,
                            csiphy_dev->csiphy_clk, csiphy_dev->num_clk, 1);
  } else if (csiphy_dev->hw_dts_version >= CSIPHY_VERSION_V30) {
    if (!csiphy_dev->clk_mux_mem || !csiphy_dev->clk_mux_io) {
      pr_err("%s clk mux mem %pK io %pK\n", __func__, csiphy_dev->clk_mux_mem,
             csiphy_dev->clk_mux_io);
      rc = -ENOMEM;
      return rc;
    }
    csiphy_dev->clk_mux_base = ioremap(csiphy_dev->clk_mux_mem->start,
                                       resource_size(csiphy_dev->clk_mux_mem));
    if (!csiphy_dev->clk_mux_base) {
      pr_err("%s: ERROR %d\n", __func__, __LINE__);
      rc = -ENOMEM;
      return rc;
    }
#undef r
#define g(off) msm_camera_io_r(csiphybase + (off))
    pr_err("cityman_csiphy dump glbl 140=0x%x 144=0x%x 148=0x%x 14c=0x%x 150=0x%x 154=0x%x 158=0x%x 15c=0x%x 160=0x%x 164=0x%x 168=0x%x\n",
           g(0x140), g(0x144), g(0x148), g(0x14C), g(0x150), g(0x154), g(0x158), g(0x15C), g(0x160), g(0x164), g(0x168));
#undef g

    CDBG("%s:%d called\n", __func__, __LINE__);
    rc = msm_cam_clk_enable(&csiphy_dev->pdev->dev, csiphy_clk_info,
                            csiphy_dev->csiphy_clk, csiphy_dev->num_clk, 1);
  } else {
    pr_err("%s: ERROR Invalid CSIPHY Version %d", __func__, __LINE__);
    rc = -EINVAL;
    return rc;
  }

  CDBG("%s:%d called\n", __func__, __LINE__);
  if (rc < 0) {
    pr_err("%s: csiphy clk enable failed\n", __func__);
    csiphy_dev->ref_count--;
    iounmap(csiphy_dev->base);
    csiphy_dev->base = NULL;
    return rc;
  }
  CDBG("%s:%d called\n", __func__, __LINE__);

  enable_irq(csiphy_dev->irq->start);

  msm_csiphy_reset(csiphy_dev);

  CDBG("%s:%d called\n", __func__, __LINE__);

  if (csiphy_dev->hw_dts_version == CSIPHY_VERSION_V30)
    csiphy_dev->hw_version = msm_camera_io_r(
        csiphy_dev->base +
        csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_hw_version_addr);
  else
    csiphy_dev->hw_version = csiphy_dev->hw_dts_version;

  CDBG("%s:%d called csiphy_dev->hw_version 0x%x\n", __func__, __LINE__,
       csiphy_dev->hw_version);
  csiphy_dev->csiphy_state = CSIPHY_POWER_UP;
  return 0;

csiphy_enable_regulator_gdscr_fail:
  rc = msm_camera_enable_vreg(&csiphy_dev->pdev->dev, csiphy_vreg_info,
                              ARRAY_SIZE(csiphy_vreg_info), NULL, 0,
                              &csiphy_dev->csi_vdd, 0);
csiphy_vreg_enable_fail:
  rc = msm_camera_config_vreg(&csiphy_dev->pdev->dev, csiphy_vreg_info,
                              ARRAY_SIZE(csiphy_vreg_info), NULL, 0,
                              &csiphy_dev->csi_vdd, 0);
csiphy_vreg_config_fail:
  iounmap(csiphy_dev->base);
  csiphy_dev->base = NULL;
  return rc;
}
#else
static int msm_csiphy_init(struct csiphy_device *csiphy_dev) {
  int rc = 0;
  if (csiphy_dev == NULL) {
    pr_err("%s: csiphy_dev NULL\n", __func__);
    rc = -ENOMEM;
    return rc;
  }

  CDBG("%s:%d called\n", __func__, __LINE__);
  if (csiphy_dev->csiphy_state == CSIPHY_POWER_UP) {
    pr_err("%s: csiphy invalid state %d\n", __func__, csiphy_dev->csiphy_state);
    rc = -EINVAL;
    return rc;
  }
  CDBG("%s:%d called\n", __func__, __LINE__);

  if (csiphy_dev->ref_count++) {
    CDBG("%s csiphy refcount = %d\n", __func__, csiphy_dev->ref_count);
    return rc;
  }
  CDBG("%s:%d called\n", __func__, __LINE__);

  csiphy_dev->base =
      ioremap(csiphy_dev->mem->start, resource_size(csiphy_dev->mem));
  if (!csiphy_dev->base) {
    pr_err("%s: csiphy_dev->base NULL\n", __func__);
    csiphy_dev->ref_count--;
    rc = -ENOMEM;
    return rc;
  }

  rc = msm_camera_config_vreg(&csiphy_dev->pdev->dev, csiphy_vreg_info,
                              ARRAY_SIZE(csiphy_vreg_info), NULL, 0,
                              &csiphy_dev->csi_vdd, 1);
  if (rc < 0) {
    pr_err("%s: regulator config failed\n", __func__);
    goto csiphy_vreg_config_fail;
  }
  rc = msm_camera_enable_vreg(&csiphy_dev->pdev->dev, csiphy_vreg_info,
                              ARRAY_SIZE(csiphy_vreg_info), NULL, 0,
                              &csiphy_dev->csi_vdd, 1);
  if (rc < 0) {
    pr_err("%s: regulator enable failed\n", __func__);
    goto csiphy_vreg_enable_fail;
  }

  csiphy_dev->reg_ptr =
      regulator_get(&(csiphy_dev->pdev->dev), "qcom,gdscr-vdd");
  if (IS_ERR_OR_NULL(csiphy_dev->reg_ptr)) {
    pr_err(" %s: Failed in getting TOP gdscr regulator handle", __func__);
  } else {
    rc = regulator_enable(csiphy_dev->reg_ptr);
    if (rc) {
      pr_err(" %s: regulator enable failed for GDSCR\n", __func__);
      goto csiphy_enable_regulator_gdscr_fail;
    }
  }

  if (csiphy_dev->hw_dts_version <= CSIPHY_VERSION_V22) {
    CDBG("%s:%d called\n", __func__, __LINE__);
    rc = msm_cam_clk_enable(&csiphy_dev->pdev->dev, csiphy_clk_info,
                            csiphy_dev->csiphy_clk, csiphy_dev->num_clk, 1);
  } else if (csiphy_dev->hw_dts_version >= CSIPHY_VERSION_V30) {
    if (!csiphy_dev->clk_mux_mem || !csiphy_dev->clk_mux_io) {
      pr_err("%s clk mux mem %pK io %pK\n", __func__, csiphy_dev->clk_mux_mem,
             csiphy_dev->clk_mux_io);
      rc = -ENOMEM;
      return rc;
    }
    csiphy_dev->clk_mux_base = ioremap(csiphy_dev->clk_mux_mem->start,
                                       resource_size(csiphy_dev->clk_mux_mem));
    if (!csiphy_dev->clk_mux_base) {
      pr_err("%s: ERROR %d\n", __func__, __LINE__);
      rc = -ENOMEM;
      return rc;
    }
    CDBG("%s:%d called\n", __func__, __LINE__);
    rc = msm_cam_clk_enable(&csiphy_dev->pdev->dev, csiphy_clk_info,
                            csiphy_dev->csiphy_clk, csiphy_dev->num_clk, 1);
  } else {
    pr_err("%s: ERROR Invalid CSIPHY Version %d", __func__, __LINE__);
    rc = -EINVAL;
    return rc;
  }

  CDBG("%s:%d called\n", __func__, __LINE__);
  if (rc < 0) {
    pr_err("%s: csiphy clk enable failed\n", __func__);
    csiphy_dev->ref_count--;
    iounmap(csiphy_dev->base);
    csiphy_dev->base = NULL;
    return rc;
  }
  CDBG("%s:%d called\n", __func__, __LINE__);

  msm_csiphy_reset(csiphy_dev);

  CDBG("%s:%d called\n", __func__, __LINE__);

  if (csiphy_dev->hw_dts_version == CSIPHY_VERSION_V30)
    csiphy_dev->hw_version = msm_camera_io_r(
        csiphy_dev->base +
        csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_hw_version_addr);
  else
    csiphy_dev->hw_version = csiphy_dev->hw_dts_version;

  CDBG("%s:%d called csiphy_dev->hw_version 0x%x\n", __func__, __LINE__,
       csiphy_dev->hw_version);
  csiphy_dev->csiphy_state = CSIPHY_POWER_UP;
  return 0;

csiphy_enable_regulator_gdscr_fail:
  rc = msm_camera_enable_vreg(&csiphy_dev->pdev->dev, csiphy_vreg_info,
                              ARRAY_SIZE(csiphy_vreg_info), NULL, 0,
                              &csiphy_dev->csi_vdd, 0);
csiphy_vreg_enable_fail:
  rc = msm_camera_config_vreg(&csiphy_dev->pdev->dev, csiphy_vreg_info,
                              ARRAY_SIZE(csiphy_vreg_info), NULL, 0,
                              &csiphy_dev->csi_vdd, 0);
csiphy_vreg_config_fail:
  iounmap(csiphy_dev->base);
  csiphy_dev->base = NULL;
  return rc;
}
#endif

#if DBG_CSIPHY
static int msm_csiphy_release(struct csiphy_device *csiphy_dev, void *arg) {
  int i = 0;
  struct msm_camera_csi_lane_params *csi_lane_params;
  uint16_t csi_lane_mask;
  csi_lane_params = (struct msm_camera_csi_lane_params *)arg;

  cancel_delayed_work_sync(&cityman_csiphy_late);

  if (!csiphy_dev || !csiphy_dev->ref_count) {
    pr_err("%s csiphy dev NULL / ref_count ZERO\n", __func__);
    return 0;
  }

  if (csiphy_dev->csiphy_state != CSIPHY_POWER_UP) {
    pr_err("%s: csiphy invalid state %d\n", __func__, csiphy_dev->csiphy_state);
    return -EINVAL;
  }

  if (csiphy_dev->hw_version < CSIPHY_VERSION_V30) {
    csiphy_dev->lane_mask[csiphy_dev->pdev->id] = 0;
    for (i = 0; i < 4; i++)
      msm_camera_io_w(
          0x0, csiphy_dev->base +
                   csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg2_addr +
                   0x40 * i);
  } else {
    if (!csi_lane_params) {
      pr_err("%s:%d failed: csi_lane_params %pK\n", __func__, __LINE__,
             csi_lane_params);
      return -EINVAL;
    }
    csi_lane_mask = (csi_lane_params->csi_lane_mask & 0x1F);

    CDBG("%s csiphy_params, lane assign 0x%x mask = 0x%x\n", __func__,
         csi_lane_params->csi_lane_assign, csi_lane_params->csi_lane_mask);

    if (!csi_lane_mask)
      csi_lane_mask = 0x1f;

    csiphy_dev->lane_mask[csiphy_dev->pdev->id] &= ~(csi_lane_mask);
    i = 0;
    while (csi_lane_mask) {
      if (csi_lane_mask & 0x1) {
        msm_camera_io_w(
            0x0,
            csiphy_dev->base +
                csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg2_addr +
                0x40 * i);
        msm_camera_io_w(
            0x0,
            csiphy_dev->base +
                csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_misc1_addr +
                0x40 * i);
        msm_camera_io_w(
            0x0, csiphy_dev->base +
                     csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_test_imp +
                     0x40 * i);
      }
      csi_lane_mask >>= 1;
      i++;
    }
  }

  if (--csiphy_dev->ref_count) {
    CDBG("%s csiphy refcount = %d\n", __func__, csiphy_dev->ref_count);
    return 0;
  }

  msm_camera_io_w(
      0x0, csiphy_dev->base +
               csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnck_cfg2_addr);
  msm_camera_io_w(
      0x0, csiphy_dev->base +
               csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_glbl_pwr_cfg_addr);

  disable_irq(csiphy_dev->irq->start);

  if (csiphy_dev->hw_dts_version <= CSIPHY_VERSION_V22) {
    msm_cam_clk_enable(&csiphy_dev->pdev->dev, csiphy_clk_info,
                       csiphy_dev->csiphy_clk, csiphy_dev->num_clk, 0);
  } else if (csiphy_dev->hw_dts_version >= CSIPHY_VERSION_V30) {
    msm_cam_clk_enable(&csiphy_dev->pdev->dev, csiphy_clk_info,
                       csiphy_dev->csiphy_clk, csiphy_dev->num_clk, 0);
    iounmap(csiphy_dev->clk_mux_base);
  }

  msm_camera_enable_vreg(&csiphy_dev->pdev->dev, csiphy_vreg_info,
                         ARRAY_SIZE(csiphy_vreg_info), NULL, 0,
                         &csiphy_dev->csi_vdd, 0);
  msm_camera_config_vreg(&csiphy_dev->pdev->dev, csiphy_vreg_info,
                         ARRAY_SIZE(csiphy_vreg_info), NULL, 0,
                         &csiphy_dev->csi_vdd, 0);

  if (!IS_ERR_OR_NULL(csiphy_dev->reg_ptr)) {
    regulator_disable(csiphy_dev->reg_ptr);
    regulator_put(csiphy_dev->reg_ptr);
  }

  iounmap(csiphy_dev->base);
  csiphy_dev->base = NULL;
  csiphy_dev->csiphy_state = CSIPHY_POWER_DOWN;
  return 0;
}
#else
static int msm_csiphy_release(struct csiphy_device *csiphy_dev, void *arg) {
  int i = 0;
  struct msm_camera_csi_lane_params *csi_lane_params;
  uint16_t csi_lane_mask;
  csi_lane_params = (struct msm_camera_csi_lane_params *)arg;

  cancel_delayed_work_sync(&cityman_csiphy_late);

  if (!csiphy_dev || !csiphy_dev->ref_count) {
    pr_err("%s csiphy dev NULL / ref_count ZERO\n", __func__);
    return 0;
  }

  if (csiphy_dev->csiphy_state != CSIPHY_POWER_UP) {
    pr_err("%s: csiphy invalid state %d\n", __func__, csiphy_dev->csiphy_state);
    return -EINVAL;
  }

  if (csiphy_dev->hw_version < CSIPHY_VERSION_V30) {
    csiphy_dev->lane_mask[csiphy_dev->pdev->id] = 0;
    for (i = 0; i < 4; i++)
      msm_camera_io_w(
          0x0, csiphy_dev->base +
                   csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg2_addr +
                   0x40 * i);
  } else {
    if (!csi_lane_params) {
      pr_err("%s:%d failed: csi_lane_params %pK\n", __func__, __LINE__,
             csi_lane_params);
      return -EINVAL;
    }
    csi_lane_mask = (csi_lane_params->csi_lane_mask & 0x1F);

    CDBG("%s csiphy_params, lane assign 0x%x mask = 0x%x\n", __func__,
         csi_lane_params->csi_lane_assign, csi_lane_params->csi_lane_mask);

    if (!csi_lane_mask)
      csi_lane_mask = 0x1f;

    csiphy_dev->lane_mask[csiphy_dev->pdev->id] &= ~(csi_lane_mask);
    i = 0;
    while (csi_lane_mask) {
      if (csi_lane_mask & 0x1) {
        msm_camera_io_w(
            0x0,
            csiphy_dev->base +
                csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_cfg2_addr +
                0x40 * i);
        msm_camera_io_w(
            0x0,
            csiphy_dev->base +
                csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_misc1_addr +
                0x40 * i);
        msm_camera_io_w(
            0x0, csiphy_dev->base +
                     csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnn_test_imp +
                     0x40 * i);
      }
      csi_lane_mask >>= 1;
      i++;
    }
  }

  if (--csiphy_dev->ref_count) {
    CDBG("%s csiphy refcount = %d\n", __func__, csiphy_dev->ref_count);
    return 0;
  }

  msm_camera_io_w(
      0x0, csiphy_dev->base +
               csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_lnck_cfg2_addr);
  msm_camera_io_w(
      0x0, csiphy_dev->base +
               csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_glbl_pwr_cfg_addr);

  if (csiphy_dev->hw_dts_version <= CSIPHY_VERSION_V22) {
    msm_cam_clk_enable(&csiphy_dev->pdev->dev, csiphy_clk_info,
                       csiphy_dev->csiphy_clk, csiphy_dev->num_clk, 0);
  } else if (csiphy_dev->hw_dts_version >= CSIPHY_VERSION_V30) {
    msm_cam_clk_enable(&csiphy_dev->pdev->dev, csiphy_clk_info,
                       csiphy_dev->csiphy_clk, csiphy_dev->num_clk, 0);
    iounmap(csiphy_dev->clk_mux_base);
  }

  msm_camera_enable_vreg(&csiphy_dev->pdev->dev, csiphy_vreg_info,
                         ARRAY_SIZE(csiphy_vreg_info), NULL, 0,
                         &csiphy_dev->csi_vdd, 0);
  msm_camera_config_vreg(&csiphy_dev->pdev->dev, csiphy_vreg_info,
                         ARRAY_SIZE(csiphy_vreg_info), NULL, 0,
                         &csiphy_dev->csi_vdd, 0);

  if (!IS_ERR_OR_NULL(csiphy_dev->reg_ptr)) {
    regulator_disable(csiphy_dev->reg_ptr);
    regulator_put(csiphy_dev->reg_ptr);
  }

  iounmap(csiphy_dev->base);
  csiphy_dev->base = NULL;
  csiphy_dev->csiphy_state = CSIPHY_POWER_DOWN;
  return 0;
}

#endif
static int32_t msm_csiphy_cmd(struct csiphy_device *csiphy_dev, void *arg) {
  int rc = 0;
  struct csiphy_cfg_data *cdata = (struct csiphy_cfg_data *)arg;
  struct msm_camera_csiphy_params csiphy_params;
  struct msm_camera_csi_lane_params csi_lane_params;
  if (!csiphy_dev || !cdata) {
    pr_err("%s: csiphy_dev NULL\n", __func__);
    return -EINVAL;
  }
  switch (cdata->cfgtype) {
  case CSIPHY_INIT:
    rc = msm_csiphy_init(csiphy_dev);
    break;
  case CSIPHY_CFG:
    if (copy_from_user(&csiphy_params, (void *)cdata->cfg.csiphy_params,
                       sizeof(struct msm_camera_csiphy_params))) {
      pr_err("%s: %d failed\n", __func__, __LINE__);
      rc = -EFAULT;
      break;
    }
    rc = msm_csiphy_lane_config(csiphy_dev, &csiphy_params);
    break;
  case CSIPHY_RELEASE:
    if (copy_from_user(&csi_lane_params, (void *)cdata->cfg.csi_lane_params,
                       sizeof(struct msm_camera_csi_lane_params))) {
      pr_err("%s: %d failed\n", __func__, __LINE__);
      rc = -EFAULT;
      break;
    }
    rc = msm_csiphy_release(csiphy_dev, &csi_lane_params);
    break;
  default:
    pr_err("%s: %d failed\n", __func__, __LINE__);
    rc = -ENOIOCTLCMD;
    break;
  }
  return rc;
}

static int32_t msm_csiphy_get_subdev_id(struct csiphy_device *csiphy_dev,
                                        void *arg) {
  uint32_t *subdev_id = (uint32_t *)arg;
  if (!subdev_id) {
    pr_err("%s:%d failed\n", __func__, __LINE__);
    return -EINVAL;
  }
  *subdev_id = csiphy_dev->pdev->id;
  pr_debug("%s:%d subdev_id %d\n", __func__, __LINE__, *subdev_id);
  return 0;
}

static long msm_csiphy_subdev_ioctl(struct v4l2_subdev *sd, unsigned int cmd,
                                    void *arg) {
  int rc = -ENOIOCTLCMD;
  struct csiphy_device *csiphy_dev = v4l2_get_subdevdata(sd);
  CDBG("%s:%d id %d\n", __func__, __LINE__, csiphy_dev->pdev->id);
  mutex_lock(&csiphy_dev->mutex);
  switch (cmd) {
  case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
    rc = msm_csiphy_get_subdev_id(csiphy_dev, arg);
    break;
  case VIDIOC_MSM_CSIPHY_IO_CFG:
    rc = msm_csiphy_cmd(csiphy_dev, arg);
    break;
  case VIDIOC_MSM_CSIPHY_RELEASE:
  case MSM_SD_SHUTDOWN:
    rc = msm_csiphy_release(csiphy_dev, arg);
    break;
  default:
    pr_err_ratelimited("%s: command not found\n", __func__);
  }
  mutex_unlock(&csiphy_dev->mutex);
  CDBG("%s:%d\n", __func__, __LINE__);
  return rc;
}

#ifdef CONFIG_COMPAT
static long msm_csiphy_subdev_do_ioctl(struct file *file, unsigned int cmd,
                                       void *arg) {
  struct video_device *vdev = video_devdata(file);
  struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
  struct csiphy_cfg_data32 *u32 = (struct csiphy_cfg_data32 *)arg;
  struct csiphy_cfg_data csiphy_data;

  switch (cmd) {
  case VIDIOC_MSM_CSIPHY_IO_CFG32:
    cmd = VIDIOC_MSM_CSIPHY_IO_CFG;
    csiphy_data.cfgtype = u32->cfgtype;
    csiphy_data.cfg.csiphy_params = compat_ptr(u32->cfg.csiphy_params);
    return msm_csiphy_subdev_ioctl(sd, cmd, &csiphy_data);
  default:
    return msm_csiphy_subdev_ioctl(sd, cmd, arg);
  }
}

static long msm_csiphy_subdev_fops_ioctl(struct file *file, unsigned int cmd,
                                         unsigned long arg) {
  return video_usercopy(file, cmd, arg, msm_csiphy_subdev_do_ioctl);
}
#endif

static const struct v4l2_subdev_internal_ops msm_csiphy_internal_ops;

static struct v4l2_subdev_core_ops msm_csiphy_subdev_core_ops = {
    .g_chip_ident = &msm_csiphy_subdev_g_chip_ident,
    .ioctl = &msm_csiphy_subdev_ioctl,
};

static const struct v4l2_subdev_ops msm_csiphy_subdev_ops = {
    .core = &msm_csiphy_subdev_core_ops,
};

static int msm_csiphy_get_clk_info(struct csiphy_device *csiphy_dev,
                                   struct platform_device *pdev) {
  uint32_t count;
  int i, rc;
  uint32_t rates[CSIPHY_NUM_CLK_MAX];

  struct device_node *of_node;
  of_node = pdev->dev.of_node;

  count = of_property_count_strings(of_node, "clock-names");
  csiphy_dev->num_clk = count;

  CDBG("%s: count = %d\n", __func__, count);
  if (count == 0) {
    pr_err("%s: no clocks found in device tree, count=%d", __func__, count);
    return 0;
  }

  if (count > CSIPHY_NUM_CLK_MAX) {
    pr_err("%s: invalid count=%d, max is %d\n", __func__, count,
           CSIPHY_NUM_CLK_MAX);
    return -EINVAL;
  }

  for (i = 0; i < count; i++) {
    rc = of_property_read_string_index(of_node, "clock-names", i,
                                       &(csiphy_clk_info[i].clk_name));
    CDBG("%s: clock-names[%d] = %s\n", __func__, i,
         csiphy_clk_info[i].clk_name);
    if (rc < 0) {
      pr_err("%s:%d, failed\n", __func__, __LINE__);
      return rc;
    }
  }
  rc = of_property_read_u32_array(of_node, "qcom,clock-rates", rates, count);
  if (rc < 0) {
    pr_err("%s:%d, failed", __func__, __LINE__);
    return rc;
  }
  for (i = 0; i < count; i++) {
    csiphy_clk_info[i].clk_rate = (rates[i] == 0) ? (long)-1 : rates[i];
    if (!strcmp(csiphy_clk_info[i].clk_name, "csiphy_timer_src_clk")) {
      CDBG("%s:%d, copy csiphy_timer_src_clk", __func__, __LINE__);
      csiphy_dev->csiphy_max_clk = rates[i];
      csiphy_dev->csiphy_clk_index = i;
    }
    CDBG("%s: clk_rate[%d] = %ld\n", __func__, i, csiphy_clk_info[i].clk_rate);
  }
  return 0;
}

static int csiphy_probe(struct platform_device *pdev) {
  struct csiphy_device *new_csiphy_dev;
  uint32_t csi_vdd_voltage = 0;
  int rc = 0;

  new_csiphy_dev = kzalloc(sizeof(struct csiphy_device), GFP_KERNEL);
  if (!new_csiphy_dev) {
    pr_err("%s: no enough memory\n", __func__);
    return -ENOMEM;
  }
  new_csiphy_dev->is_3_1_20nm_hw = 0;
  new_csiphy_dev->ctrl_reg = NULL;
  new_csiphy_dev->ctrl_reg = kzalloc(sizeof(struct csiphy_ctrl_t), GFP_KERNEL);
  if (!new_csiphy_dev->ctrl_reg) {
    pr_err("%s:%d kzalloc failed\n", __func__, __LINE__);
    return -ENOMEM;
  }
  v4l2_subdev_init(&new_csiphy_dev->msm_sd.sd, &msm_csiphy_subdev_ops);
  v4l2_set_subdevdata(&new_csiphy_dev->msm_sd.sd, new_csiphy_dev);
  platform_set_drvdata(pdev, &new_csiphy_dev->msm_sd.sd);

  mutex_init(&new_csiphy_dev->mutex);

  if (pdev->dev.of_node) {
    of_property_read_u32((&pdev->dev)->of_node, "cell-index", &pdev->id);
    CDBG("%s: device id = %d\n", __func__, pdev->id);
  }

  rc = msm_csiphy_get_clk_info(new_csiphy_dev, pdev);
  if (rc < 0) {
    pr_err("%s: msm_csiphy_get_clk_info() failed", __func__);
    return -EFAULT;
  }

  rc = of_property_read_u32((&pdev->dev)->of_node, "qcom,csi-vdd-voltage",
                            &csi_vdd_voltage);
  if (rc < 0) {
    pr_err("%s:%d failed to read qcom,csi-vdd-voltage\n", __func__, __LINE__);
    return rc;
  }
  CDBG("%s:%d reading mipi_csi_vdd is %d\n", __func__, __LINE__,
       csi_vdd_voltage);

  csiphy_vreg_info[0].min_voltage = csi_vdd_voltage;
  csiphy_vreg_info[0].max_voltage = csi_vdd_voltage;

  new_csiphy_dev->mem =
      platform_get_resource_byname(pdev, IORESOURCE_MEM, "csiphy");
  if (!new_csiphy_dev->mem) {
    pr_err("%s: no mem resource?\n", __func__);
    rc = -ENODEV;
    goto csiphy_no_resource;
  }
  new_csiphy_dev->irq =
      platform_get_resource_byname(pdev, IORESOURCE_IRQ, "csiphy");
  if (!new_csiphy_dev->irq) {
    pr_err("%s: no irq resource?\n", __func__);
    rc = -ENODEV;
    goto csiphy_no_resource;
  }
  new_csiphy_dev->io =
      request_mem_region(new_csiphy_dev->mem->start,
                         resource_size(new_csiphy_dev->mem), pdev->name);
  if (!new_csiphy_dev->io) {
    pr_err("%s: no valid mem region\n", __func__);
    rc = -EBUSY;
    goto csiphy_no_resource;
  }

  rc = request_irq(new_csiphy_dev->irq->start, msm_csiphy_irq,
                   IRQF_TRIGGER_RISING, "csiphy", new_csiphy_dev);
  if (rc < 0) {
    release_mem_region(new_csiphy_dev->mem->start,
                       resource_size(new_csiphy_dev->mem));
    pr_err("%s: irq request fail\n", __func__);
    rc = -EBUSY;
    goto csiphy_no_resource;
  }
  disable_irq(new_csiphy_dev->irq->start);

  new_csiphy_dev->clk_mux_mem =
      platform_get_resource_byname(pdev, IORESOURCE_MEM, "csiphy_clk_mux");
  if (new_csiphy_dev->clk_mux_mem) {
    new_csiphy_dev->clk_mux_io =
        request_mem_region(new_csiphy_dev->clk_mux_mem->start,
                           resource_size(new_csiphy_dev->clk_mux_mem),
                           new_csiphy_dev->clk_mux_mem->name);
    if (!new_csiphy_dev->clk_mux_io)
      pr_err("%s: ERROR %d\n", __func__, __LINE__);
  }

  new_csiphy_dev->pdev = pdev;
  new_csiphy_dev->msm_sd.sd.internal_ops = &msm_csiphy_internal_ops;
  new_csiphy_dev->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
  snprintf(new_csiphy_dev->msm_sd.sd.name,
           ARRAY_SIZE(new_csiphy_dev->msm_sd.sd.name), "msm_csiphy");
  media_entity_init(&new_csiphy_dev->msm_sd.sd.entity, 0, NULL, 0);
  new_csiphy_dev->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
  new_csiphy_dev->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_CSIPHY;
  new_csiphy_dev->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x4;
  msm_sd_register(&new_csiphy_dev->msm_sd);

  if (of_device_is_compatible(new_csiphy_dev->pdev->dev.of_node,
                              "qcom,csiphy-v2.0")) {
    new_csiphy_dev->ctrl_reg->csiphy_reg = csiphy_v2_0;
    new_csiphy_dev->hw_dts_version = CSIPHY_VERSION_V20;
  } else if (of_device_is_compatible(new_csiphy_dev->pdev->dev.of_node,
                                     "qcom,csiphy-v2.2")) {
    new_csiphy_dev->ctrl_reg->csiphy_reg = csiphy_v2_2;
    new_csiphy_dev->hw_dts_version = CSIPHY_VERSION_V22;
  } else if (of_device_is_compatible(new_csiphy_dev->pdev->dev.of_node,
                                     "qcom,csiphy-v3.0")) {
    new_csiphy_dev->ctrl_reg->csiphy_reg = csiphy_v3_0;
    new_csiphy_dev->hw_dts_version = CSIPHY_VERSION_V30;
  } else if (of_device_is_compatible(new_csiphy_dev->pdev->dev.of_node,
                                     "qcom,csiphy-v3.1")) {
    new_csiphy_dev->ctrl_reg->csiphy_reg = csiphy_v3_1;
    new_csiphy_dev->hw_dts_version = CSIPHY_VERSION_V31;
  } else if (of_device_is_compatible(new_csiphy_dev->pdev->dev.of_node,
                                     "qcom,csiphy-v3.1.1")) {
    new_csiphy_dev->ctrl_reg->csiphy_reg = csiphy_v3_1;
    new_csiphy_dev->hw_dts_version = CSIPHY_VERSION_V31;
    new_csiphy_dev->is_3_1_20nm_hw = 1;
  } else if (of_device_is_compatible(new_csiphy_dev->pdev->dev.of_node,
                                     "qcom,csiphy-v3.2")) {
    new_csiphy_dev->ctrl_reg->csiphy_reg = csiphy_v3_2;
    new_csiphy_dev->hw_dts_version = CSIPHY_VERSION_V32;
  } else {
    pr_err("%s:%d, invalid hw version : 0x%x", __func__, __LINE__,
           new_csiphy_dev->hw_dts_version);
    return -EINVAL;
  }

  msm_csiphy_v4l2_subdev_fops = v4l2_subdev_fops;
#ifdef CONFIG_COMPAT
  msm_csiphy_v4l2_subdev_fops.compat_ioctl32 = msm_csiphy_subdev_fops_ioctl;
#endif
  new_csiphy_dev->msm_sd.sd.devnode->fops = &msm_csiphy_v4l2_subdev_fops;
  new_csiphy_dev->csiphy_state = CSIPHY_POWER_DOWN;
  return 0;

csiphy_no_resource:
  mutex_destroy(&new_csiphy_dev->mutex);
  kfree(new_csiphy_dev->ctrl_reg);
  kfree(new_csiphy_dev);
  return 0;
}

static const struct of_device_id msm_csiphy_dt_match[] = {
    {.compatible = "qcom,csiphy"}, {}};

MODULE_DEVICE_TABLE(of, msm_csiphy_dt_match);

static struct platform_driver csiphy_driver = {
    .probe = csiphy_probe,
    .driver =
        {
            .name = MSM_CSIPHY_DRV_NAME,
            .owner = THIS_MODULE,
            .of_match_table = msm_csiphy_dt_match,
        },
};

static int __init msm_csiphy_init_module(void) {
  return platform_driver_register(&csiphy_driver);
}

static void __exit msm_csiphy_exit_module(void) {
  platform_driver_unregister(&csiphy_driver);
}

module_init(msm_csiphy_init_module);
module_exit(msm_csiphy_exit_module);
MODULE_DESCRIPTION("MSM CSIPHY driver");
MODULE_LICENSE("GPL v2");
