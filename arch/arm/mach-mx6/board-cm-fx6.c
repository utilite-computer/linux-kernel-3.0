/*
 * CompuLab CM-FX6 module support
 *
 * Copyright (C) 2013 CompuLab, Ltd.
 * Author: Igor Grinberg <grinberg@compulab.co.il>
 *
 * Derived from board-mx6q_arm2.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 */

#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/spi/ads7846.h>
#include <linux/spi/scf0403.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/i2c/at24.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mxc_asrc.h>
#include <linux/mfd/mxc-hdmi-core.h>
#include <linux/igb.h>
#include <linux/gpio-i2cmux.h>
#include <linux/rtc/rtc-em3027.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/esdhc.h>
#include <mach/iomux-mx6q.h>
#include <mach/iomux-mx6dl.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <media/tvp5150.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-cm-fx6-q.h"
#include "board-cm-fx6-dl.h"

/* GPIO PIN, sort by board/PORT/BIT */
#define CM_FX6_SATA_PWREN		IMX_GPIO_NR(1, 28)
#define CM_FX6_SATA_VDDC_CTRL		IMX_GPIO_NR(1, 30)
#define CM_FX6_ADS7846_PENDOWN		IMX_GPIO_NR(2, 15)
#define CM_FX6_SATA_LDO_EN		IMX_GPIO_NR(2, 16)
#define CM_FX6_ECSPI1_CS0		IMX_GPIO_NR(2, 30)

#define CM_FX6_WIFI_NPD			IMX_GPIO_NR(7, 12)
#define CM_FX6_WIFI_NRESET		IMX_GPIO_NR(6, 16)

#define CM_FX6_GREEN_LED		IMX_GPIO_NR(2, 31)
#define CM_FX6_ECSPI1_CS1		IMX_GPIO_NR(3, 19)
#define CM_FX6_SATA_nSTANDBY1		IMX_GPIO_NR(3, 20)
#define CM_FX6_SATA_PHY_SLP		IMX_GPIO_NR(3, 23)
#define CM_FX6_ECSPI2_CS2		IMX_GPIO_NR(3, 24)
#define CM_FX6_ECSPI2_CS3		IMX_GPIO_NR(3, 25)
#define CM_FX6_SATA_STBY_REQ		IMX_GPIO_NR(3, 29)
#define CM_FX6_SATA_nSTANDBY2		IMX_GPIO_NR(5, 2)
#define CM_FX6_SATA_nRSTDLY		IMX_GPIO_NR(6, 6)
#define CM_FX6_SATA_PWLOSS_INT		IMX_GPIO_NR(6, 31)
#define CM_FX6_USB_HUB_RST		IMX_GPIO_NR(7, 8)

#define SB_FX6_HIMAX_PENDOWN		IMX_GPIO_NR(1, 4)
#define SB_FX6_ETH_RST			IMX_GPIO_NR(1, 26)
#define SB_FX6_USB_OTG_PWR		IMX_GPIO_NR(3, 22)
#define SB_FX6_SD3_WP			IMX_GPIO_NR(7, 0)
#define SB_FX6_SD3_CD			IMX_GPIO_NR(7, 1)
#define SB_FX6_GPIO_EXT_BASE		IMX_GPIO_NR(8, 0)
#define SB_FX6_PCIE_MUX_PWR		IMX_GPIO_NR(8, 4)
#define SB_FX6_LCD_RST			IMX_GPIO_NR(8, 11)

#define SB_FX6M_EM3027_IRQ		IMX_GPIO_NR(1, 1)
#define SB_FX6M_DVI_DDC_SEL		IMX_GPIO_NR(1, 2)
#define SB_FX6M_DVI_HPD			IMX_GPIO_NR(1, 4)

static u32 board_rev;

extern char *soc_reg_id;
extern char *pu_reg_id;

#if defined(CONFIG_MTD_NAND_GPMI_NAND)
static int cm_fx6_gpmi_nand_init_pads(void)
{
	iomux_v3_cfg_t *nand_pads = NULL;
	u32 nand_pads_cnt;

	if (cpu_is_mx6q()) {
		nand_pads = cm_fx6_q_gpmi_nand;
		nand_pads_cnt = ARRAY_SIZE(cm_fx6_q_gpmi_nand);
	} else if (cpu_is_mx6dl()) {
		nand_pads = cm_fx6_dl_gpmi_nand;
		nand_pads_cnt = ARRAY_SIZE(cm_fx6_dl_gpmi_nand);

	}
	BUG_ON(!nand_pads);
	return mxc_iomux_v3_setup_multiple_pads(nand_pads, nand_pads_cnt);
}

static struct mtd_partition cm_fx6_nand_partitions[] = {
	{
		.name	= "linux",
		.offset	= 0,
		.size	= SZ_8M,
	},
	{
		.name	= "rootfs",
		.offset	= MTDPART_OFS_APPEND,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct gpmi_nand_platform_data cm_fx6_gpmi_nand_pdata = {
	.platform_init		= cm_fx6_gpmi_nand_init_pads,
	.min_prop_delay_in_ns	= 5,
	.max_prop_delay_in_ns	= 9,
	.max_chip_count		= 1,
	.partitions		= cm_fx6_nand_partitions,
	.partition_count	= ARRAY_SIZE(cm_fx6_nand_partitions),
	.enable_bbt		= 1,
};

static void __init cm_fx6_nand_init(void)
{
	struct platform_device * pdev;

	pdev = imx6q_add_gpmi(&cm_fx6_gpmi_nand_pdata);
	if (IS_ERR(pdev))
		pr_err("%s: GPMI NAND registration failed: %ld\n",
		       __func__, PTR_ERR(pdev));
}
#else
static inline void cm_fx6_nand_init(void) {}
#endif

static const struct anatop_thermal_platform_data cm_fx6_anatop_thermal_data = {
	.name = "anatop_thermal",
};

static inline void cm_fx6_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(3, NULL);
	imx6q_add_imx_uart(1, NULL);
	imx6q_add_imx_uart(4, NULL);
}

#define BMCR_PDOWN 0x0800 /* PHY Powerdown */

static int cm_fx6_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

	/* Ar8031 phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	phy_write(phydev, 0xd, 0x3);
	phy_write(phydev, 0xe, 0x805d);
	phy_write(phydev, 0xd, 0x4003);
	val = phy_read(phydev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(phydev, 0xe, val);

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x8016);
	phy_write(phydev, 0xd, 0x4007);
	val = phy_read(phydev, 0xe);

	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, 0xe, val);

	/* introduce tx clock delay */
	phy_write(phydev, 0x1d, 0x5);
	val = phy_read(phydev, 0x1e);
	val |= 0x0100;
	phy_write(phydev, 0x1e, val);

	/* check phy power */
	val = phy_read(phydev, 0x0);
	if (val & BMCR_PDOWN)
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));

	return 0;
}

static int cm_fx6_fec_power_hibernate(struct phy_device *phydev)
{
	unsigned short val;

	/*set AR8031 debug reg 0xb to hibernate power*/
	phy_write(phydev, 0x1d, 0xb);
	val = phy_read(phydev, 0x1e);

	val |= 0x8000;
	phy_write(phydev, 0x1e, val);

	return 0;
}

static struct fec_platform_data cm_fx6_fec_data = {
	.init			= cm_fx6_fec_phy_init,
	.power_hibernate	= cm_fx6_fec_power_hibernate,
	.phy			= PHY_INTERFACE_MODE_RGMII,
};

#if defined(CONFIG_SPI_IMX) || defined(CONFIG_SPI_IMX_MODULE)
static int cm_fx6_spi0_cs[] = {
	CM_FX6_ECSPI1_CS0,	/* CS0 */
	CM_FX6_ECSPI1_CS1,	/* CS1 */
};

static int cm_fx6_spi1_cs[] = {
	CM_FX6_ECSPI2_CS2,	/* CS0 */
	CM_FX6_ECSPI2_CS3,	/* CS1 */
};

static const struct spi_imx_master cm_fx6_spi0_data = {
	.chipselect	= cm_fx6_spi0_cs,
	.num_chipselect	= ARRAY_SIZE(cm_fx6_spi0_cs),
};

static const struct spi_imx_master cm_fx6_spi1_data = {
	.chipselect	= cm_fx6_spi1_cs,
	.num_chipselect	= ARRAY_SIZE(cm_fx6_spi1_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition cm_fx6_spi_flash_partitions[] = {
	{
		.name	= "uboot",
		.offset	= 0,
		.size	= SZ_512K + SZ_256K,
	}, {
		.name	= "uboot environment",
		.offset	= MTDPART_OFS_APPEND,
		.size	= SZ_256K,
	}, {
		.name	= "reserved",
		.offset	= MTDPART_OFS_APPEND,
		.size	= MTDPART_SIZ_FULL,
	},
};

/*
 * The default flash on cm-fx6 is "sst25vf016b".
 * It is JEDEC compliant, so we don't specify the .type feild below.
 */
static struct flash_platform_data cm_fx6_spi_flash_data = {
	.name		= "spi_flash",
	.parts		= cm_fx6_spi_flash_partitions,
	.nr_parts	= ARRAY_SIZE(cm_fx6_spi_flash_partitions),
};
#endif

#if defined(CONFIG_TOUCHSCREEN_ADS7846) || \
	defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)
static struct regulator_consumer_supply ads7846_supplies[] = {
	REGULATOR_SUPPLY("vcc", "spi0.1"),
};

static struct regulator_init_data ads7846_vcc_data = {
	.num_consumer_supplies = ARRAY_SIZE(ads7846_supplies),
	.consumer_supplies = ads7846_supplies,
};

static struct fixed_voltage_config ads7846_reg_config = {
	.supply_name	= "ads7846_vcc",
	.microvolts	= 3300000,
	.gpio		= -EINVAL,
	.init_data	= &ads7846_vcc_data,
};

static struct platform_device ads7846_reg_pdev = {
	.name		= "reg-fixed-voltage",
	.id		= 10,
	.dev		= {
		.platform_data = &ads7846_reg_config,
	},
};

static struct ads7846_platform_data ads7846_config = {
	.x_max			= 0x0fff,
	.y_max			= 0x0fff,
	.x_plate_ohms		= 180,
	.pressure_max		= 255,
	.debounce_max		= 30,
	.debounce_tol		= 10,
	.debounce_rep		= 1,
	.gpio_pendown		= CM_FX6_ADS7846_PENDOWN,
	.keep_vref_on		= 1,
};

static void __init ads7846_init(void)
{
	int err;

	err = platform_device_register(&ads7846_reg_pdev);
	if (err)
		pr_err("%s: ADS7846 regulator register failed: %d\n",
		       __func__, err);
}
#else /* CONFIG_TOUCHSCREEN_ADS7846 */
static inline void ads7846_init(void) {}
#endif /* CONFIG_TOUCHSCREEN_ADS7846 */

static struct spi_board_info cm_fx6_spi0_board_info[] __initdata = {
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
	{
		.modalias	= "m25p80",
		.max_speed_hz	= 20000000,
		.bus_num	= 0,
		.chip_select	= 0,
		.platform_data	= &cm_fx6_spi_flash_data,
	},
#endif /* CONFIG_MTD_M25P80 */
#if defined(CONFIG_TOUCHSCREEN_ADS7846) || \
	defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)
	{
		.modalias	= "ads7846",
		.max_speed_hz	= 1500000,
		.bus_num	= 0,
		.chip_select	= 1,
		.irq		= gpio_to_irq(CM_FX6_ADS7846_PENDOWN),
		.platform_data	= &ads7846_config,
	},
#endif /* CONFIG_TOUCHSCREEN_ADS7846 */
};

static void __init spi_register_bus_binfo(int busnum,
					  const struct spi_imx_master *spidata,
					  struct spi_board_info *info,
					  int info_size)
{
	int err;
	struct platform_device * pdev;

	pdev = imx6q_add_ecspi(busnum, spidata);
	if (IS_ERR(pdev))
		pr_err("%s: SPI bus %d register failed: %ld\n",
		       __func__, busnum, PTR_ERR(pdev));

	if (info) {
		err = spi_register_board_info(info, info_size);
		if (err)
			pr_err("%s: SPI%d board info register failed: %d\n",
			       __func__, busnum, err);
	}
}

static void __init cm_fx6_spi_init(void)
{
	ads7846_init();
	spi_register_bus_binfo(0, &cm_fx6_spi0_data,
			       cm_fx6_spi0_board_info,
			       ARRAY_SIZE(cm_fx6_spi0_board_info));
	spi_register_bus_binfo(1, &cm_fx6_spi1_data, NULL, 0);
}

#if defined(CONFIG_LCD_SCF0403) || defined(CONFIG_LCD_SCF0403_MODULE)
static struct scf0403_pdata sb_fx6_scf0403_lcd_data = {
	.reset_gpio	= SB_FX6_LCD_RST,
};

static struct spi_board_info sb_fx6_scf0403_lcd_info = {
	.modalias               = "scf0403",
	.max_speed_hz           = 1000000,
	.bus_num                = 1,
	.chip_select            = 1,
	.platform_data          = &sb_fx6_scf0403_lcd_data,
};

static void sb_fx6_scf0403_lcd_init(void)
{
	struct spi_master *master;

	master = spi_busnum_to_master(1);
	if (!master) {
		pr_err("%s: SPI1 master get failed!\n", __func__);
		return;
	}

	if (!spi_new_device(master, &sb_fx6_scf0403_lcd_info))
		pr_err("%s: scf0403 registration failed on SPI1\n", __func__);

	spi_master_put(master);
}
#else /* CONFIG_LCD_SCF0403 */
static inline void sb_fx6_scf0403_lcd_init(void) {}
#endif /* CONFIG_LCD_SCF0403 */

#else /* CONFIG_SPI_IMX */
static inline void cm_fx6_spi_init(void) {}
static inline void sb_fx6_scf0403_lcd_init(void) {}
#endif /* CONFIG_SPI_IMX */

#if defined(CONFIG_TOUCHSCREEN_HIMAX) || \
	defined(CONFIG_TOUCHSCREEN_HIMAX_MODULE)
static void __init sb_fx6_himax_ts_init(void)
{
	int err;

	err = gpio_request_one(SB_FX6_HIMAX_PENDOWN, GPIOF_IN, "himax_pen");
	if (err) {
		pr_err("CM-FX6: Couldn't obtain gpio for himax_pen: %d\n", err);
		return;
	}
	gpio_export(SB_FX6_HIMAX_PENDOWN, 0);
}
#else /* CONFIG_TOUCHSCREEN_HIMAX */
static inline void sb_fx6_himax_ts_init(void) {}
#endif /* CONFIG_TOUCHSCREEN_HIMAX */

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
#if defined(CONFIG_FB_MXC_HDMI) || defined(CONFIG_FB_MXC_HDMI_MODULE)
static void cm_fx6_hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;
	int max_ipu_id = cpu_is_mx6q() ? 1 : 0;

	if ((ipu_id > max_ipu_id) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2 * ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

static struct fsl_mxc_hdmi_platform_data cm_fx6_hdmi_data = {
	.init = cm_fx6_hdmi_init,
};

static struct fsl_mxc_hdmi_core_platform_data cm_fx6_hdmi_core_data = {
	.ipu_id		= 0,
	.disp_id	= 1,
};

static struct platform_device mxc_hdmi_audio_device = {
	.name           = "mxc_hdmi_audio",
	.id             = -1,
};

static void __init cm_fx6_init_hdmi(void)
{
	struct platform_device * pdev;

	pdev = imx6q_add_mxc_hdmi_core(&cm_fx6_hdmi_core_data);
	if (IS_ERR(pdev)) {
		pr_err("%s: HDMI core register failed: %ld\n",
		       __func__, PTR_ERR(pdev));
		return;
	}

	pdev = imx6q_add_mxc_hdmi(&cm_fx6_hdmi_data);
	if (IS_ERR(pdev))
		pr_err("%s: HDMI register failed: %ld\n",
		       __func__, PTR_ERR(pdev));

}

#else /* CONFIG_FB_MXC_HDMI */
static inline void cm_fx6_init_hdmi(void) {}
#endif /* CONFIG_FB_MXC_HDMI */

static struct ipuv3_fb_platform_data cm_fx6_lcd_pdata = {
	.disp_dev		= "lcd",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB24,
	.mode_str		= "KD050C-WVGA",
	.default_bpp		= 24,
	.int_clk		= false,
};

static struct ipuv3_fb_platform_data cm_fx6_dvi_pdata = {
	.disp_dev		= "dvi",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB32,
	.mode_str		= "1280x800@60",
	.default_bpp		= 32,
	.int_clk		= false,
};

static struct ipuv3_fb_platform_data cm_fx6_hdmi_pdata = {
	.disp_dev		= "hdmi",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB32,
	.mode_str		= "1920x1080@60",
	.default_bpp		= 32,
	.int_clk		= false,
};

static struct ipuv3_fb_platform_data cm_fx6_ldb0_pdata = {
	.disp_dev		= "ldb",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB666,
	.mode_str		= "1366x768M-18@60",
	.default_bpp		= 18,
	.int_clk		= false,
};

static struct ipuv3_fb_platform_data cm_fx6_ldb1_pdata = {
	.disp_dev		= "ldb",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB666,
	.mode_str		= "1280x800M-18@60",
	.default_bpp		= 18,
	.int_clk		= false,
};

static struct ipuv3_fb_platform_data *sb_fx6_fb_data[] = {
	&cm_fx6_lcd_pdata,
	&cm_fx6_hdmi_pdata,
	&cm_fx6_ldb0_pdata,
	&cm_fx6_ldb1_pdata,
};

static struct ipuv3_fb_platform_data *sb_fx6m_fb_data[] = {
	&cm_fx6_hdmi_pdata,
	&cm_fx6_dvi_pdata,
};

static struct ipuv3_fb_platform_data **baseboard_fb_data;
static int baseboard_fb_data_size;

static struct fsl_mxc_lcd_platform_data cm_fx6_lcdif_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
	.default_ifmt	= IPU_PIX_FMT_RGB24,
};

static struct fsl_mxc_ldb_platform_data cm_fx6_ldb0_data = {
	.ipu_id		= 1,
	.disp_id	= 0,
	.ext_ref	= 1,
	.mode		= LDB_SEP0,
	.sec_ipu_id	= 1,
	.sec_disp_id	= 1,
};

static struct fsl_mxc_ldb_platform_data cm_fx6_ldb1_data = {
	.ipu_id		= 1,
	.disp_id	= 1,
	.ext_ref	= 1,
	.mode		= LDB_SEP1,
	.sec_ipu_id	= 1,
	.sec_disp_id	= 0,
};

static int __init early_set_lcd_type(char *p)
{
	if (p && !strcmp(p, "dataimage")) {
		cm_fx6_lcd_pdata.interface_pix_fmt = IPU_PIX_FMT_RGB666;
		cm_fx6_lcd_pdata.mode_str = "SCF04-WVGA";
		cm_fx6_lcd_pdata.default_bpp = 0;
	}

	return 0;
}
early_param("cm_fx6_lcd", early_set_lcd_type);

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
		.rev		= 4,
		.csi_clk[0]	= "clko_clk",
	}, {
		.rev		= 4,
		.csi_clk[0]	= "clko_clk",
	},
};

static void __init cm_fx6_init_ipu(void)
{
	struct platform_device * pdev;

	pdev = imx6q_add_ipuv3(0, &ipu_data[0]);
	if (IS_ERR(pdev)) {
		pr_err("%s: IPU 0 register failed: %ld\n",
		       __func__, PTR_ERR(pdev));
		return;
	}

	if (cpu_is_mx6q()) {
		pdev = imx6q_add_ipuv3(1, &ipu_data[1]);
		if (IS_ERR(pdev)) {
			pr_err("%s: IPU 1 register failed: %ld\n",
				   __func__, PTR_ERR(pdev));
		return;
		}
	}
}

static int __init cm_fx6_init_display(void)
{
	int i;
	struct platform_device * pdev;

	pdev = imx6q_add_vdoa();
	if (IS_ERR(pdev))
		pr_err("%s: VDOA register failed: %ld\n",
		       __func__, PTR_ERR(pdev));

	pdev = imx6q_add_lcdif(&cm_fx6_lcdif_data);
	if (IS_ERR(pdev))
		pr_err("%s: lcd interface register failed: %ld\n",
		       __func__, PTR_ERR(pdev));

	for (i = 0; i < baseboard_fb_data_size; i++) {
		pdev = imx6q_add_ipuv3fb(i, baseboard_fb_data[i]);
		if (IS_ERR(pdev))
			pr_err("%s: fb%d register failed: %ld\n",
				   __func__, i, PTR_ERR(pdev));
	}

	return 0;
}
#else /* CONFIG_FB_MXC_SYNC_PANEL */
static inline void cm_fx6_init_display(void) {}
#endif /* CONFIG_FB_MXC_SYNC_PANEL */

#if defined(CONFIG_I2C_IMX) || defined(CONFIG_I2C_IMX_MODULE)
/* EEPROM layout */
#define EEPROM_1ST_MAC_OFF		4
#define EEPROM_BOARD_NAME_OFF		128
#define EEPROM_BOARD_NAME_LEN		16

static int eeprom_read(struct memory_accessor *mem_acc, unsigned char *buf,
		       int offset, int size, const char* objname)
{
	ssize_t ret;

	ret = mem_acc->read(mem_acc, buf, offset, size);
	if (ret != size) {
		pr_warn("CM-FX6: EEPROM %s read failed: %d\n", objname, ret);
		return ret;
	}

	return 0;
}

static void eeprom_read_mac_address(struct memory_accessor *mem_acc,
				    unsigned char *mac)
{
	char *objname = "MAC address";

	if (eeprom_read(mem_acc, mac, EEPROM_1ST_MAC_OFF, ETH_ALEN, objname))
		memset(mac, 0, ETH_ALEN);
}

static void eeprom_read_board_name(struct memory_accessor *mem_acc,
				   unsigned char *name)
{
	char *objname = "board name";

	if (eeprom_read(mem_acc, name, EEPROM_BOARD_NAME_OFF,
			EEPROM_BOARD_NAME_LEN, objname))
		memset(name, 0, EEPROM_BOARD_NAME_LEN);
}

static void cm_fx6_eeprom_setup(struct memory_accessor *mem_acc, void *context)
{
	eeprom_read_mac_address(mem_acc, cm_fx6_fec_data.mac);
	imx6_init_fec(cm_fx6_fec_data);
}

static struct at24_platform_data cm_fx6_eeprom_pdata = {
        .byte_len       = 256,
        .page_size      = 16,
	.setup		= cm_fx6_eeprom_setup,
};

static void baseboard_i2c_device_register(int busnum,
					  struct i2c_board_info *info,
					  char *device_name)
{
	struct i2c_adapter *i2c_adapter;

	i2c_adapter = i2c_get_adapter(busnum);
	if (!i2c_adapter) {
		pr_err("%s: I2C%d adapter get failed!\n", __func__, busnum);
		return;
	}

	if (!i2c_new_device(i2c_adapter, info))
		pr_err("%s: %s registration failed on I2C%d!\n",
		       __func__, device_name, busnum);

	i2c_put_adapter(i2c_adapter);
}

#if defined(CONFIG_FB_MXC_EDID) || defined(CONFIG_FB_MXC_EDID_MODULE)
static int sb_fx6_dvi_update(void)
{
	/* always connected */
	return 1;
}

static void sb_fx6m_dvi_init(void)
{
	int err;

	err = gpio_request(SB_FX6M_DVI_HPD, "dvi detect");
	if (err)
		pr_err("%s > error %d\n", __func__, err);

	gpio_direction_input(SB_FX6M_DVI_HPD);
}

static int sb_fx6m_dvi_update(void)
{
	return gpio_get_value(SB_FX6M_DVI_HPD);
}

static struct fsl_mxc_dvi_platform_data baseboard_dvi_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
	.init		= sb_fx6m_dvi_init,
	.update		= sb_fx6m_dvi_update,
};

static struct i2c_board_info baseboard_dvi_info = {
	I2C_BOARD_INFO("mxc_dvi", 0x50),
	.irq = gpio_to_irq(SB_FX6M_DVI_HPD),
	.platform_data = &baseboard_dvi_data,
};

static void baseboard_dvi_register(void)
{
	baseboard_i2c_device_register(4, &baseboard_dvi_info, "DVI");
}
#else /* CONFIG_FB_MXC_EDID */
static inline void baseboard_dvi_register(void) {}
#endif /* CONFIG_FB_MXC_EDID */

static struct i2c_board_info cm_fx6_i2c1_board_info[] __initdata = {
#if defined(CONFIG_FB_MXC_HDMI) || defined(CONFIG_FB_MXC_HDMI_MODULE)
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
#endif /* CONFIG_FB_MXC_HDMI */
};

static struct em3027_platform_data sb_fx6m_rtc_pdata = {
	.charger_resistor_sel = EM3027_TRICKLE_CHARGER_1_5K,
};

static struct i2c_board_info sb_fx6m_rtc_info = {
	I2C_BOARD_INFO("em3027", 0x56),
	.irq = gpio_to_irq(SB_FX6M_EM3027_IRQ),
	.platform_data = &sb_fx6m_rtc_pdata,
};

static void sb_fx6m_rtc_register(void)
{
	int err;

	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_pad(MX6Q_PAD_GPIO_1__GPIO_1_1);
	else if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_pad(MX6DL_PAD_GPIO_1__GPIO_1_1);

	err = gpio_request_one(SB_FX6M_EM3027_IRQ, GPIOF_IN, "rtc irq");
	if (err) {
		pr_err("%s: failed requesting GPIO%d: %d\n",
		       __func__, SB_FX6M_EM3027_IRQ, err);
		return;
	}

	baseboard_i2c_device_register(3, &sb_fx6m_rtc_info, "em3027 rtc");
}

static struct i2c_board_info cm_fx6_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("at24", 0x50),
		.platform_data = &cm_fx6_eeprom_pdata,
	},
#if defined(CONFIG_SND_SOC_CM_FX6) || defined(CONFIG_SND_SOC_CM_FX6_MODULE)
	{
		/* wm8731 audio codec */
		I2C_BOARD_INFO("wm8731", 0x1a),
	},
#endif /* CONFIG_SND_SOC_CM_FX6 */
};

static struct imx_pcie_platform_data baseboard_pcie_data = {
	.pcie_pwr_en	= -EINVAL,
	.pcie_rst	= SB_FX6_ETH_RST,
	.pcie_wake_up	= -EINVAL,
	.pcie_dis	= -EINVAL,
};

#if defined(CONFIG_TOUCHSCREEN_HIMAX) || \
	defined(CONFIG_TOUCHSCREEN_HIMAX_MODULE)
static struct i2c_board_info sb_fx6_himax_ts_info = {
	I2C_BOARD_INFO("hx8526-a", 0x4A),
	.irq = gpio_to_irq(SB_FX6_HIMAX_PENDOWN),
};

static void sb_fx6_himax_ts_register(void)
{
	baseboard_i2c_device_register(3, &sb_fx6_himax_ts_info, "Himax TS");
}
#else /* CONFIG_TOUCHSCREEN_HIMAX */
static inline void sb_fx6_himax_ts_register(void) {}
#endif /* CONFIG_TOUCHSCREEN_HIMAX */

static void sb_fx6_ldb_register(void)
{
	struct platform_device * pdev;

	pdev = imx6q_add_ldb(0, &cm_fx6_ldb0_data);
	if (IS_ERR(pdev))
		pr_err("%s: ldb interface register failed: %ld\n",
			__func__, PTR_ERR(pdev));

	pdev = imx6q_add_ldb(1, &cm_fx6_ldb1_data);
	if (IS_ERR(pdev))
		pr_err("%s: ldb1 interface register failed: %ld\n",
			__func__, PTR_ERR(pdev));
}

#if defined(CONFIG_GPIO_PCA953X) || defined(CONFIG_GPIO_PCA953X_MODULE)
static struct pca953x_platform_data sb_fx6_gpio_ext_pdata = {
	.gpio_base = SB_FX6_GPIO_EXT_BASE,
};

static struct i2c_board_info sb_fx6_gpio_ext_info = {
	I2C_BOARD_INFO("pca9555", 0x26),
	.platform_data = &sb_fx6_gpio_ext_pdata,
};

static void sb_fx6_gpio_ext_register(void)
{
	baseboard_i2c_device_register(3, &sb_fx6_gpio_ext_info, "GPIO ext");
}
#else /* CONFIG_GPIO_PCA953X */
static inline void sb_fx6_gpio_ext_register(void) {}
#endif /* CONFIG_GPIO_PCA953X */

static const struct esdhc_platform_data cm_fx6_sd1_data __initconst = {
	.always_present		= 1,
	.keep_power_at_suspend	= 1,
};

/* The default configuration is set for SB-FX6m */
static struct esdhc_platform_data baseboard_sd3_data = {
	.cd_type		= ESDHC_CD_NONE,
	.cd_gpio		= -EINVAL,
	.wp_gpio		= -EINVAL,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
	.always_present		= 1,
};

static void baseboard_sd3_init(void)
{
	iomux_v3_cfg_t *sd3_pads;
	iomux_v3_cfg_t *sd3_full_pads	= cm_fx6_q_sd3_full_200mhz;
	iomux_v3_cfg_t *sd3_half_pads	= cm_fx6_q_sd3_half_200mhz;
	unsigned int sd3_pads_cnt;
	unsigned int sd3_full_pads_cnt	= ARRAY_SIZE(cm_fx6_q_sd3_full_200mhz);
	unsigned int sd3_half_pads_cnt	= ARRAY_SIZE(cm_fx6_q_sd3_half_200mhz);

	if (cpu_is_mx6dl()) {
		sd3_full_pads		= cm_fx6_dl_sd3_full_200mhz;
		sd3_half_pads		= cm_fx6_dl_sd3_half_200mhz;
		sd3_full_pads_cnt	= ARRAY_SIZE(cm_fx6_dl_sd3_full_200mhz);
		sd3_half_pads_cnt	= ARRAY_SIZE(cm_fx6_dl_sd3_half_200mhz);
	}

	sd3_pads	= sd3_half_pads;
	sd3_pads_cnt	= sd3_half_pads_cnt;

	if (baseboard_sd3_data.support_8bit) {
		sd3_pads	= sd3_full_pads;
		sd3_pads_cnt	= sd3_full_pads_cnt;
	}

	mxc_iomux_v3_setup_multiple_pads(sd3_pads, sd3_pads_cnt);
	imx6q_add_sdhci_usdhc_imx(2, &baseboard_sd3_data);
}

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	},
};

static void __init sb_fx6_eval_camera_init(void)
{
	/* Capture devices init */
	imx6q_add_v4l2_capture(0, &capture_data[0]);
}
void (*sb_fx6_camera_init)(void);

static void sb_fx6_init(void)
{
	pr_info("CM-FX6: Detected SB-FX6 (Eval) base board\n");

	baseboard_sd3_data.cd_type = ESDHC_CD_GPIO;
	baseboard_sd3_data.cd_gpio = SB_FX6_SD3_CD;
	baseboard_sd3_data.wp_gpio = SB_FX6_SD3_WP;
	baseboard_sd3_data.always_present = 0;

	baseboard_dvi_data.init = NULL;
	baseboard_dvi_data.update = sb_fx6_dvi_update;
	/* 0x7f - fake address, as SB-FX6 rev 1.0 does not support DVI DDC */
	baseboard_dvi_info.addr = 0x7f;
	baseboard_dvi_info.irq = 0;

	baseboard_pcie_data.pcie_pwr_en = SB_FX6_PCIE_MUX_PWR;

	baseboard_fb_data = sb_fx6_fb_data;
	baseboard_fb_data_size = ARRAY_SIZE(sb_fx6_fb_data);

	imx6q_add_imx_snvs_rtc();
	sb_fx6_gpio_ext_register();
	sb_fx6_scf0403_lcd_init();
	sb_fx6_himax_ts_init();
	sb_fx6_himax_ts_register();
	sb_fx6_ldb_register();
	sb_fx6_camera_init = sb_fx6_eval_camera_init;
}

static void sb_fx6m_init(void)
{
	pr_info("CM-FX6: Detected SB-FX6m (Utilite) base board\n");

	baseboard_fb_data = sb_fx6m_fb_data;
	baseboard_fb_data_size = ARRAY_SIZE(sb_fx6m_fb_data);

	/*for Utilite only HDMI to IPU1 */
	if (cpu_is_mx6q())
		cm_fx6_hdmi_core_data.ipu_id = 1;

	sb_fx6m_rtc_register();
}

static struct igb_platform_data baseboard_igb_pdata;

static void baseboard_eeprom_setup(struct memory_accessor *mem_acc,
				   void *context)
{
	unsigned char baseboard[EEPROM_BOARD_NAME_LEN];

	eeprom_read_board_name(mem_acc, baseboard);
	if (strncmp(baseboard, "SB-FX6m", EEPROM_BOARD_NAME_LEN) == 0)
		sb_fx6m_init();
	else
		sb_fx6_init();


	eeprom_read_mac_address(mem_acc, baseboard_igb_pdata.mac_address);
	igb_set_platform_data(&baseboard_igb_pdata);

	baseboard_sd3_init();
	imx6q_add_pcie(&baseboard_pcie_data);
}

static struct at24_platform_data baseboard_eeprom_pdata = {
	.byte_len	= 256,
	.page_size	= 16,
	.setup		= baseboard_eeprom_setup,
};

/* For MX6Q:
 * GPR1 bit19 and bit20 meaning:
 * Bit19:       0 - Enable mipi to IPU1 CSI0
 *                      virtual channel is fixed to 0
 *              1 - Enable parallel interface to IPU1 CSI0
 * Bit20:       0 - Enable mipi to IPU2 CSI1
 *                      virtual channel is fixed to 3
 *              1 - Enable parallel interface to IPU2 CSI1
 * IPU1 CSI1 directly connect to mipi csi2,
 *      virtual channel is fixed to 1
 * IPU2 CSI0 directly connect to mipi csi2,
 *      virtual channel is fixed to 2
 *
 * For MX6DL:
 * GPR1 bit 21 and GPR13 bit 0-5, RM has detail information
 */
static void cm_fx6_csi0_init(void)
{
	if (cpu_is_mx6q())
		mxc_iomux_set_gpr_register(1, 19, 1, 1);
	else if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 0, 3, 4);
}

static struct fsl_mxc_camera_platform_data fsl_camera_data[] = {
	{
		.mclk = 24000000,
		.mclk_source = 0,
		.csi = 0,
		.io_init = cm_fx6_csi0_init,
	},
};

static struct tvp5150_platform_data cm_fx6_tvp5150_pdata = {
	.platform_data = &fsl_camera_data[0],
};

static struct i2c_board_info cm_fx6_i2c0c3_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("at24", 0x50),
		.platform_data = &baseboard_eeprom_pdata,
	},
	{
		I2C_BOARD_INFO("tvp5150", 0x5c),
		.platform_data = &cm_fx6_tvp5150_pdata,
	},
};

static const unsigned sb_fx6m_i2cmux_gpios[] = {
	SB_FX6M_DVI_DDC_SEL,
};

static const unsigned sb_fx6m_i2cmux_values[] = {
	0, 1,
};

static struct gpio_i2cmux_platform_data sb_fx6m_i2cmux_data = {
	.parent		= 0,	/* multiplex I2C-0 */
	.base_nr	= 3,	/* create I2C-3+ */
	.values		= sb_fx6m_i2cmux_values,
	.n_values	= ARRAY_SIZE(sb_fx6m_i2cmux_values),
	.gpios		= sb_fx6m_i2cmux_gpios,
	.n_gpios	= ARRAY_SIZE(sb_fx6m_i2cmux_gpios),
	.idle		= GPIO_I2CMUX_NO_IDLE,
};

static struct platform_device sb_fx6m_i2cmux = {
	.name	= "gpio-i2cmux",
	.id	= -1,
	.dev	= {
		.platform_data = &sb_fx6m_i2cmux_data,
	},
};

static struct imxi2c_platform_data cm_fx6_i2c0_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data cm_fx6_i2c1_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data cm_fx6_i2c2_data = {
	.bitrate = 400000,
};

static void __init i2c_register_bus_binfo(int busnum,
					  struct imxi2c_platform_data *i2cdata,
					  struct i2c_board_info *info,
					  int info_size)
{
	int err;
	struct platform_device *pdev;

	if (i2cdata) {
		pdev = imx6q_add_imx_i2c(busnum, i2cdata);
		if (IS_ERR(pdev))
			pr_err("%s: I2C%d register failed: %ld\n",
			       __func__, busnum, PTR_ERR(pdev));
	}

	if (info) {
		err = i2c_register_board_info(busnum, info, info_size);
		if (err)
			pr_err("%s: I2C%d board info register failed: %d\n",
			       __func__, busnum, err);
	}
}

static void __init cm_fx6_i2c_init(void)
{
	/* register the physical bus 0 w/o any devices */
	i2c_register_bus_binfo(0, &cm_fx6_i2c0_data, NULL, 0);

	i2c_register_bus_binfo(1, &cm_fx6_i2c1_data, cm_fx6_i2c1_board_info,
			       ARRAY_SIZE(cm_fx6_i2c1_board_info));
	i2c_register_bus_binfo(2, &cm_fx6_i2c2_data, cm_fx6_i2c2_board_info,
			       ARRAY_SIZE(cm_fx6_i2c2_board_info));

	/* I2C multiplexing: I2C-0 --> I2C-3, I2C-4 */
	platform_device_register(&sb_fx6m_i2cmux);

	/* register the virtual bus 3 */
	i2c_register_bus_binfo(3, NULL, cm_fx6_i2c0c3_board_info,
			       ARRAY_SIZE(cm_fx6_i2c0c3_board_info));

}
#else /* CONFIG_I2C_IMX */
static inline void cm_fx6_i2c_init(void) {}
#endif /* CONFIG_I2C_IMX */

#if defined(CONFIG_SND_SOC_CM_FX6) || defined(CONFIG_SND_SOC_CM_FX6_MODULE)
#define	WM8731_MCLK_FREQ	(24000000 / 2)

static int audmod_master;

static int __init early_set_audio_mode(char *p)
{
	audmod_master = 1;
	return 0;
}
early_param("audmod-mst", early_set_audio_mode);

static struct imx_ssi_platform_data cm_fx6_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data cm_fx6_audio_data;

static struct {
	struct clk *pll;
	struct clk *clock_root;
	long current_rate;

} cm_fx6_audio_clocking_data;

static int wm8731_slv_mode_init(void)
{
	struct clk *new_parent;
	struct clk *ssi_clk;

	new_parent = clk_get(NULL, "pll4");
	if (IS_ERR(new_parent)) {
		pr_err("Could not get \"pll4\" clock \n");
		return PTR_ERR(new_parent);
	}

	ssi_clk = clk_get_sys("imx-ssi.1", NULL);
	if (IS_ERR(ssi_clk)) {
		pr_err("Could not get \"imx-ssi.1\" clock \n");
		return PTR_ERR(ssi_clk);
	}

	clk_set_parent(ssi_clk, new_parent);

	cm_fx6_audio_clocking_data.pll = new_parent;
	cm_fx6_audio_clocking_data.clock_root = ssi_clk;
	cm_fx6_audio_clocking_data.current_rate = 0;

	cm_fx6_audio_data.sysclk = 0;

	return 0;
}

static int wm8731_slv_mode_clock_enable(int enable)
{
	long pll_rate;
	long rate_req;
	long rate_avail;

	if (!enable)
		return 0;

	if (cm_fx6_audio_data.sysclk == cm_fx6_audio_clocking_data.current_rate)
		return 0;

	switch (cm_fx6_audio_data.sysclk) {
		case 11289600:
			pll_rate = 632217600;
			break;

		case 12288000:
			pll_rate = 688128000;
			break;

		default:
			return -EINVAL;
	}

	rate_req = pll_rate;
	rate_avail = clk_round_rate(cm_fx6_audio_clocking_data.pll, rate_req);
	clk_set_rate(cm_fx6_audio_clocking_data.pll, rate_avail);

	rate_req = cm_fx6_audio_data.sysclk;
	rate_avail = clk_round_rate(cm_fx6_audio_clocking_data.clock_root,
				    rate_req);
	clk_set_rate(cm_fx6_audio_clocking_data.clock_root, rate_avail);

	pr_info("%s: \"imx-ssi.1\" rate = %ld (= %ld)\n",
		__func__, rate_avail, rate_req);
	cm_fx6_audio_clocking_data.current_rate = cm_fx6_audio_data.sysclk;

	return 0;
}

static int wm8731_mst_mode_init(void)
{
	long rate;
	struct clk *clko2;
	struct clk *clko;

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2)) {
		pr_err("Could not get CLKO2 clock \n");
		return PTR_ERR(clko2);
	}
	rate = clk_round_rate(clko2, WM8731_MCLK_FREQ);
	clk_set_rate(clko2, rate);

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("Could not get CLKO clock \n");
		return PTR_ERR(clko);
	}

	clk_set_parent(clko, clko2);

	rate = clk_round_rate(clko, WM8731_MCLK_FREQ);
	clk_set_rate(clko, rate);

	pr_info("%s: \"CLKO\" rate = %ld (= %d)\n",
		__func__, rate, WM8731_MCLK_FREQ);
	cm_fx6_audio_clocking_data.clock_root = clko;
	cm_fx6_audio_data.sysclk = rate;

	return 0;
}

static int wm8731_mst_mode_clock_enable(int enable)
{
	struct clk *clko = cm_fx6_audio_clocking_data.clock_root;

	if (enable)
		return clk_enable(clko);

	clk_disable(clko);
	return 0;
}

static struct platform_device cm_fx6_audio_device = {
	.name	= "imx-wm8731",
	.id	= -1,
};

static struct mxc_audio_platform_data cm_fx6_audio_data = {
	.ssi_num	= 1,
	.src_port	= 2,
	.ext_port	= 4,	/* AUDMUX: port[2] -> port[4] */
	.hp_gpio	= -1,
	.mic_gpio	= -1,
	.init		= wm8731_slv_mode_init,
	.clock_enable	= wm8731_slv_mode_clock_enable,
	.codec_name	= "wm8731-slv-mode",
};

static void __init cm_fx6_init_audio(void)
{
	if (audmod_master) {
		/*
		 * For the master mode to work correctly, cm-fx6 should have:
		 * R105 populated and R104 removed.
		 */
		if (cpu_is_mx6q())
			mxc_iomux_v3_setup_pad(MX6Q_PAD_GPIO_5__CCM_CLKO);
		else if (cpu_is_mx6dl())
			mxc_iomux_v3_setup_pad(MX6DL_PAD_GPIO_5__CCM_CLKO);

		cm_fx6_audio_data.init = wm8731_mst_mode_init;
		cm_fx6_audio_data.clock_enable = wm8731_mst_mode_clock_enable;
		cm_fx6_audio_data.codec_name = "wm8731-mst-mode";
	}

	mxc_register_device(&cm_fx6_audio_device, &cm_fx6_audio_data);
	imx6q_add_imx_ssi(1, &cm_fx6_ssi_pdata);
}
#else
static inline void cm_fx6_init_audio(void) {}
#endif /* CONFIG_SND_SOC_CM_FX6 */

static void cm_fx6_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(SB_FX6_USB_OTG_PWR, 1);
	else
		gpio_set_value(SB_FX6_USB_OTG_PWR, 0);
}

static void __init cm_fx6_usb_hub_reset(void)
{
	int err;

	err = gpio_request_one(CM_FX6_USB_HUB_RST,
			       GPIOF_OUT_INIT_LOW, "usb hub rst");
	if (err) {
		pr_err("CM-FX6: usb hub rst gpio request failed: %d\n", err);
		return;
	}

	udelay(10);
	gpio_set_value(CM_FX6_USB_HUB_RST, 1);
	msleep(1);
}

static void __init cm_fx6_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	/* disable external charger detect,
	 * or it will affect signal quality at dp.
	 */
	ret = gpio_request_one(SB_FX6_USB_OTG_PWR,
			       GPIOF_OUT_INIT_LOW, "usb-pwr");
	if (ret)
		pr_err("%s: USB_OTG_PWR gpio request failed: %d\n",
		       __func__, ret);

	cm_fx6_usb_hub_reset();

	mxc_iomux_set_gpr_register(1, 13, 1, 0);

	mx6_set_otghost_vbus_func(cm_fx6_usbotg_vbus);
}

#if defined(CONFIG_SATA_AHCI_PLATFORM) || \
	defined(CONFIG_SATA_AHCI_PLATFORM_MODULE)
static struct clk *sata_clk;

static struct gpio sata_issd_gpios[] = {
	/* The order of the GPIOs in the array is important! */
	{ CM_FX6_SATA_PHY_SLP,	 GPIOF_OUT_INIT_LOW,	"sata phy slp" },
	{ CM_FX6_SATA_nRSTDLY,	 GPIOF_OUT_INIT_LOW,	"sata nrst" },
	{ CM_FX6_SATA_PWREN,	 GPIOF_OUT_INIT_LOW,	"sata pwren" },
	{ CM_FX6_SATA_nSTANDBY1, GPIOF_OUT_INIT_LOW,	"sata nstndby1" },
	{ CM_FX6_SATA_nSTANDBY2, GPIOF_OUT_INIT_LOW,	"sata nstndby2" },
	{ CM_FX6_SATA_LDO_EN,	 GPIOF_OUT_INIT_LOW,	"sata ldo en" },
};

static void cm_fx6_sata_power_on(bool on)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sata_issd_gpios); i++) {
		gpio_set_value(sata_issd_gpios[i].gpio, on ? 1 : 0);
		udelay(100);
	}
}

/* HW Initialization, if return 0, initialization is successful. */
static int cm_fx6_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int err;
	struct clk *clk;

	/* SATA PWR GPIOs */
	err = gpio_request_array(sata_issd_gpios, ARRAY_SIZE(sata_issd_gpios));
	if (err) {
		dev_err(dev, "sata power GPIOs request failed: %d\n", err);
		return err;
	}
	udelay(100);

	cm_fx6_sata_power_on(true);

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		err = PTR_ERR(sata_clk);
		dev_err(dev, "no sata clock err: %d\n", err);
		goto free_gpios;
	}

	err = clk_enable(sata_clk);
	if (err) {
		dev_err(dev, "can't enable sata clock err: %d\n", err);
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFF) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		err = PTR_ERR(clk);
		dev_err(dev, "no ahb clock err: %d\n", err);
		goto release_sata_clk;
	}

	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

	err = sata_init(addr, tmpdata);
	if (!err)
		return 0;

release_sata_clk:
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);
free_gpios:
	cm_fx6_sata_power_on(false);
	gpio_free_array(sata_issd_gpios, ARRAY_SIZE(sata_issd_gpios));

	return err;
}

static void cm_fx6_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);

	cm_fx6_sata_power_on(false);
	gpio_free_array(sata_issd_gpios, ARRAY_SIZE(sata_issd_gpios));
}

static struct ahci_platform_data cm_fx6_sata_pdata = {
	.init	= cm_fx6_sata_init,
	.exit	= cm_fx6_sata_exit,
};

static void __init cm_fx6_init_sata(void)
{
	struct platform_device * pdev;

	if (cpu_is_mx6q()) {
		pdev = imx6q_add_ahci(0, &cm_fx6_sata_pdata);
		if (IS_ERR(pdev))
			pr_err("%s: AHCI SATA register failed: %ld\n",
			       __func__, PTR_ERR(pdev));
	}
}
#else /* SATA_AHCI_PLATFORM */
static inline void cm_fx6_init_sata(void) {}
#endif /* SATA_AHCI_PLATFORM */

#if defined(CONFIG_BACKLIGHT_PWM) || defined(CONFIG_BACKLIGHT_PWM_MODULE)
static struct platform_pwm_backlight_data sb_fx6_pwm_backlight_data = {
	.pwm_id		= 2,
	.max_brightness	= 255,
	.dft_brightness	= 255,
	.pwm_period_ns	= 100000,
};

static void __init cm_fx6_pwm_init(void)
{
	struct platform_device * pdev;

	pdev = imx6q_add_mxc_pwm(2);
	if (IS_ERR(pdev))
		pr_err("%s: PWM 2 register failed: %ld\n",
		       __func__, PTR_ERR(pdev));

	pdev = imx6q_add_mxc_pwm_backlight(2, &sb_fx6_pwm_backlight_data);
	if (IS_ERR(pdev))
		pr_err("%s: backlight on PWM 2 register failed: %ld\n",
		       __func__, PTR_ERR(pdev));
}
#else /* CONFIG_BACKLIGHT_PWM */
static inline void cm_fx6_pwm_init() {}
#endif /* CONFIG_BACKLIGHT_PWM */

static void arm2_suspend_enter(void)
{
	/* suspend preparation */
}

static void arm2_suspend_exit(void)
{
	/* resmue resore */
}
static const struct pm_platform_data mx6_arm2_pm_data __initconst = {
	.name		= "imx_pm",
	.suspend_enter	= arm2_suspend_enter,
	.suspend_exit	= arm2_suspend_exit,
};

static struct regulator_consumer_supply cm_fx6_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.0"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
};

static struct regulator_init_data cm_fx6_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(cm_fx6_vmmc_consumers),
	.consumer_supplies = cm_fx6_vmmc_consumers,
};

static struct fixed_voltage_config cm_fx6_vmmc_reg_config = {
	.supply_name	= "vmmc",
	.microvolts	= 3300000,
	.gpio		= -1,
	.init_data	= &cm_fx6_vmmc_init,
};

static struct platform_device cm_fx6_vmmc_reg_device = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev		= {
		.platform_data = &cm_fx6_vmmc_reg_config,
	},
};

static struct mxc_mlb_platform_data mx6_arm2_mlb150_data = {
	.reg_nvcc		= NULL,
	.mlb_clk		= "mlb150_clk",
	.mlb_pll_clk		= "pll6",
};

static struct mxc_dvfs_platform_data arm2_dvfscore_data = {
	.reg_id			= "cpu_vddgp",
	.soc_id			= "cpu_vddsoc",
	.pu_id			= "cpu_vddvpu",
	.clk1_id		= "cpu_clk",
	.clk2_id		= "gpc_dvfs_clk",
	.gpc_cntr_offset	= MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset	= MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset	= MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset	= MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask		= 0x1F800,
	.prediv_offset		= 11,
	.prediv_val		= 3,
	.div3ck_mask		= 0xE0000000,
	.div3ck_offset		= 29,
	.div3ck_val		= 2,
	.emac_val		= 0x08,
	.upthr_val		= 25,
	.dnthr_val		= 9,
	.pncthr_val		= 33,
	.upcnt_val		= 10,
	.dncnt_val		= 10,
	.delay_time		= 80,
};

#if defined(CONFIG_SND_SOC_IMX_SPDIF) || \
    defined(CONFIG_SND_SOC_IMX_SPDIF_MODULE)
static int cm_fx6_spdif_clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long rate_actual;

	rate_actual = clk_round_rate(clk, rate);
	clk_set_rate(clk, rate_actual);

	return 0;
}

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx		= 1,	/* enable tx */
	.spdif_rx		= 1,	/* enable rx */
	/*
	 * spdif0_clk will be 454.7MHz divided by ccm dividers.
	 *
	 * 44.1KHz: 454.7MHz / 7 (ccm) / 23 (spdif) = 44,128 Hz ~ 0.06% error
	 * 48KHz:   454.7MHz / 4 (ccm) / 37 (spdif) = 48,004 Hz ~ 0.01% error
	 * 32KHz:   454.7MHz / 6 (ccm) / 37 (spdif) = 32,003 Hz ~ 0.01% error
	 */
	.spdif_clk_44100	= 1,    /* tx clk from spdif0_clk_root */
	.spdif_clk_48000	= 1,    /* tx clk from spdif0_clk_root */
	.spdif_div_44100	= 23,
	.spdif_div_48000	= 37,
	.spdif_div_32000	= 37,
	.spdif_rx_clk		= 0,    /* rx clk from spdif stream */
	.spdif_clk_set_rate	= cm_fx6_spdif_clk_set_rate,
	.spdif_clk		= NULL, /* spdif bus clk */
};

static void __init cm_fx6_spdif_init(void)
{
	mxc_spdif_data.spdif_core_clk = clk_get_sys("mxc_spdif.0", NULL);
	clk_put(mxc_spdif_data.spdif_core_clk);
	imx6q_add_spdif(&mxc_spdif_data);
	imx6q_add_spdif_dai();
	imx6q_add_spdif_audio_device();
}
#else
static inline void cm_fx6_spdif_init(void) {}
#endif /* CONFIG_SND_SOC_IMX_SPDIF */

#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
static struct gpio_led cm_fx6_leds[] = {
	[0] = {
		.gpio			= CM_FX6_GREEN_LED,
		.name			= "cm_fx6:green",
		.default_trigger	= "heartbeat",
		.active_low		= 0,
	},
};

static struct gpio_led_platform_data cm_fx6_led_pdata = {
	.num_leds	= ARRAY_SIZE(cm_fx6_leds),
	.leds		= cm_fx6_leds,
};

static struct platform_device cm_fx6_led_device = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data = &cm_fx6_led_pdata,
	},
};

static void __init cm_fx6_init_led(void)
{
	platform_device_register(&cm_fx6_led_device);
}
#else
static inline void cm_fx6_init_led(void) {}
#endif

#if defined(CONFIG_SND_SOC_IMX_HDMI) || defined(CONFIG_SND_SOC_IMX_HDMI_MODULE)
static void __init cm_fx6_init_hdmi_audio(void)
{
	struct platform_device * pdev;

	pdev = imx6q_add_hdmi_soc();
	if (IS_ERR(pdev)) {
		pr_err("%s: HDMI SOC audio registration failed: %ld\n",
		       __func__, PTR_ERR(pdev));
		return;
	}

	pdev = imx6q_add_hdmi_soc_dai();
	if (IS_ERR(pdev))
		pr_err("%s: HDMI SOC DAI registration failed: %ld\n",
		       __func__, PTR_ERR(pdev));

	platform_device_register(&mxc_hdmi_audio_device);

}
#else /* CONFIG_SND_SOC_IMX_HDMI */
static inline void cm_fx6_init_hdmi_audio(void) {}
#endif /* CONFIG_SND_SOC_IMX_HDMI */

static struct gpio cm_fx6_wifi_gpios[] = {
	{ CM_FX6_WIFI_NPD, GPIOF_OUT_INIT_HIGH, "wifi pdn" },
	{ CM_FX6_WIFI_NRESET, GPIOF_OUT_INIT_LOW, "wifi rstn" },
};

static void cm_fx6_init_wifi(void)
{
	int err;

	err = gpio_request_array(cm_fx6_wifi_gpios,
							 ARRAY_SIZE(cm_fx6_wifi_gpios));
	if (err) {
		pr_err("CM-FX6: failed to request wifi-gpios: %d\n", err);
	} else {
		msleep(1);
		gpio_set_value(CM_FX6_WIFI_NRESET, 1);
	}

	gpio_export(CM_FX6_WIFI_NPD, 0);
	gpio_export(CM_FX6_WIFI_NRESET, 0);
}

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits	= 4,
	.clk_map_ver	= 2,
};

static struct viv_gpu_platform_data imx6_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_64M,
};

/*
 * Set fsl_system_rev:
 * bit 0-7: Chip Revision ID
 * bit 8-11: Freescale Boards Revision ID
 *     0: Unknown or latest revision
 *     1: RevA Board
 *     2: RevB board
 *     3: RevC board
 * bit 12-19: Chip Silicon ID
 *     0x63: i.MX6 Dual/Quad
 *     0x61: i.MX6 Solo/DualLite
 *     0x60: i.MX6 SoloLite
 */
static void cm_fx6_setup_system_rev(void)
{
	/* Read Silicon information from Anatop register */
	/* The register layout:
	 * bit 16-23: Chip Silicon ID
	 * 0x60: i.MX6 SoloLite
	 * 0x61: i.MX6 Solo/DualLite
	 * 0x63: i.MX6 Dual/Quad
	 *
	 * bit 0-7: Chip Revision ID
	 * 0x00: TO1.0
	 * 0x01: TO1.1
	 * 0x02: TO1.2
	 *
	 * exp:
	 * Chip             Major    Minor
	 * i.MX6Q1.0:       6300     00
	 * i.MX6Q1.1:       6300     01
	 * i.MX6Solo1.0:    6100     00

	 * Thus the system_rev will be the following layout:
	 * | 31 - 20   | 19 - 12 | 11 - 8 | 7 - 0  |
	 * | ZERO BITS | CHIP ID | BD REV | SI REV |
	 */
	u32 fsl_system_rev = 0;

	u32 cpu_type = readl(IO_ADDRESS(ANATOP_BASE_ADDR + 0x260));

	/* Chip Silicon ID */
	fsl_system_rev = ((cpu_type >> 16) & 0xFF) << 12;
	/* Chip silicon major revision */
	fsl_system_rev |= ((cpu_type >> 8) & 0xFF) << 4;
	fsl_system_rev += 0x10;
	/* Chip silicon minor revision */
	fsl_system_rev |= cpu_type & 0xFF;

	/*
	 * Move the CompuLab board revision to a different variable,
	 * so we can use it anytime it is needed.
	 * Put the Freescale silicon revision information to the place where
	 * the userspace video libraries expect it to be.
	 */
	board_rev = system_rev;
	system_rev = fsl_system_rev;
}

#define MX6_SNVS_LPCR_REG	0x38

static void mx6_snvs_poweroff(void)
{
	void __iomem *mx6_snvs_base = MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;

	value = readl(mx6_snvs_base + MX6_SNVS_LPCR_REG);
	/* set TOP and DP_EN bits */
	value |= 0x0060;
	writel(value, mx6_snvs_base + MX6_SNVS_LPCR_REG);
}

/*
 * Board specific initialization.
 */
static void __init cm_fx6_init(void)
{
	iomux_v3_cfg_t *common_pads = NULL;
	int common_pads_cnt;

	cm_fx6_setup_system_rev();

	/*
	 * common pads: pads are non-shared with others on this board
	 * feature_pds: pads are shared with others on this board
	 */
	if (cpu_is_mx6q()) {
		common_pads = cm_fx6_q_common_pads;
		common_pads_cnt = ARRAY_SIZE(cm_fx6_q_common_pads);
	} else if (cpu_is_mx6dl()) {
		common_pads = cm_fx6_dl_pads;
		common_pads_cnt = ARRAY_SIZE(cm_fx6_dl_pads);
	}

	BUG_ON(!common_pads);
	mxc_iomux_v3_setup_multiple_pads(common_pads, common_pads_cnt);

	gp_reg_id = arm2_dvfscore_data.reg_id;
	soc_reg_id = arm2_dvfscore_data.soc_id;
	pu_reg_id = arm2_dvfscore_data.pu_id;
	cm_fx6_init_uart();

	cm_fx6_init_ipu();

	imx6q_add_imx_snvs_pwrkey();
	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_imx_caam();

	cm_fx6_i2c_init();
	cm_fx6_init_led();
	cm_fx6_init_wifi();
	cm_fx6_spi_init();

	imx6q_add_anatop_thermal_imx(1, &cm_fx6_anatop_thermal_data);

	imx6q_add_pm_imx(0, &mx6_arm2_pm_data);

	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(cm_fx6_q_sd1_half_200mhz,
					 ARRAY_SIZE(cm_fx6_q_sd1_half_200mhz));
	} else {
		mxc_iomux_v3_setup_multiple_pads(cm_fx6_dl_sd1_half_200mhz,
					 ARRAY_SIZE(cm_fx6_dl_sd1_half_200mhz));
	}

	imx6q_add_sdhci_usdhc_imx(0, &cm_fx6_sd1_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6_gpu_pdata);

	cm_fx6_init_sata();

	imx6q_add_vpu();
	cm_fx6_init_usb();
	platform_device_register(&cm_fx6_vmmc_reg_device);
	mx6_cpu_regulator_init();

	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	cm_fx6_nand_init();

	imx6q_add_dvfs_core(&arm2_dvfscore_data);

	cm_fx6_pwm_init();
	cm_fx6_spdif_init();

	imx6q_add_flexcan0(NULL);

	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
	imx6q_add_mlb150(&mx6_arm2_mlb150_data);

	imx6q_add_busfreq();

	cm_fx6_init_audio();
}

static resource_size_t cm_fx6_v4l_msize = SZ_64M;

static int __init cm_fx6_v4l_setup(char *arg)
{
	cm_fx6_v4l_msize = memparse(arg, NULL);

	pr_info("%s: cm_fx6_v4l_msize: %u\n", __func__, cm_fx6_v4l_msize);

	return 0;
}
early_param("cm_fx6_v4l_msize", cm_fx6_v4l_setup);

static int __init cm_fx6_init_v4l(void)
{
	struct platform_device *voutdev;
	resource_size_t res_mbase = 0;
	resource_size_t res_msize = cm_fx6_v4l_msize;
	phys_addr_t phys;
	int err = 0;

	if (res_msize) {
		/* memblock_alloc_base() will not return on failure (panic). */
		phys = memblock_alloc_base(res_msize, SZ_4K, SZ_1G);
		err = memblock_remove(phys, res_msize);
		if (err) {
			pr_err("%s: memblock_remove for base=%lx, size=%lx failed: %d\n",
			       __func__, (unsigned long) phys,
			       (unsigned long) res_msize, err);
			return err;
		}

		res_mbase = phys;
	}

	voutdev = imx6q_add_v4l2_output(0);
	if (IS_ERR(voutdev)) {
		pr_err("%s: imx6q_add_v4l2_output failed: %ld\n",
			   __func__, PTR_ERR(voutdev));
		return PTR_ERR(voutdev);
	}

	if (res_msize && voutdev) {
		/* dma_declare_coherent_memory returns 0 on any error */
		err = dma_declare_coherent_memory(&voutdev->dev,
					res_mbase, res_mbase, res_msize,
					DMA_MEMORY_MAP | DMA_MEMORY_EXCLUSIVE);
		if (err == 0) {
			platform_device_unregister(voutdev);
			return -ENOMEM;
		}
	}

	return 0;
}

static int __init cm_fx6_init_late(void)
{
	if (!machine_is_cm_fx6())
		return -ENODEV;

	baseboard_dvi_register();
	cm_fx6_init_hdmi();
	cm_fx6_init_display();
	cm_fx6_init_hdmi_audio();
	/*
	 * This function has to be called after
	 * all frame buffers have been registered
	 */
	if (cpu_is_mx6q())
		cm_fx6_init_v4l();

	if (sb_fx6_camera_init)
		sb_fx6_camera_init();

	return 0;
}
device_initcall_sync(cm_fx6_init_late);

#define CM_FX6_MX6Q_MIN_SOC_VOLTAGE	1250000
#define CM_FX6_MX6Q_MIN_PU_VOLTAGE	1250000
#define CM_FX6_MX6Q_MIN_CPU_VOLTAGE	1250000

static void cm_fx6_adjust_cpu_op(void)
{
	struct cpu_op *op;
	int n;

	if (cpu_is_mx6q()) {
		op = mx6_get_cpu_op(&n);
		if (!op)
			return;

		for (n--; n >= 0; n--) {
			if (op[n].soc_voltage < CM_FX6_MX6Q_MIN_SOC_VOLTAGE)
				op[n].soc_voltage = CM_FX6_MX6Q_MIN_SOC_VOLTAGE;
			if (op[n].pu_voltage < CM_FX6_MX6Q_MIN_PU_VOLTAGE)
				op[n].pu_voltage = CM_FX6_MX6Q_MIN_PU_VOLTAGE;
			if (op[n].cpu_voltage < CM_FX6_MX6Q_MIN_CPU_VOLTAGE)
				op[n].cpu_voltage = CM_FX6_MX6Q_MIN_CPU_VOLTAGE;
		}
	}
}

extern void __iomem *twd_base;

static void __init cm_fx6_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	cm_fx6_adjust_cpu_op();
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.3", NULL);
	early_console_setup(UART4_BASE_ADDR, uart_clk);
}

static struct sys_timer cm_fx6_timer = {
	.init   = cm_fx6_timer_init,
};

static void __init cm_fx6_reserve(void)
{
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	phys_addr_t phys;

	if (imx6_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(
			imx6_gpu_pdata.reserved_mem_size, SZ_4K, SZ_2G);
		memblock_remove(phys, imx6_gpu_pdata.reserved_mem_size);
		imx6_gpu_pdata.reserved_mem_base = phys;
	}
#endif
}

MACHINE_START(CM_FX6, "Compulab CM-FX6")
	.boot_params	= MX6_PHYS_OFFSET + 0x100,
	.map_io		= mx6_map_io,
	.init_irq	= mx6_init_irq,
	.init_machine	= cm_fx6_init,
	.timer		= &cm_fx6_timer,
	.reserve	= cm_fx6_reserve,
MACHINE_END
