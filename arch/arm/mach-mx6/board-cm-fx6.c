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
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
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

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-cm-fx6-q.h"
#include "board-cm-fx6-dl.h"

/* GPIO PIN, sort by PORT/BIT */
#define CM_FX6_GREEN_LED		IMX_GPIO_NR(2, 31)
#define CM_FX6_ECSPI1_CS0		IMX_GPIO_NR(2, 30)
#define CM_FX6_ECSPI1_CS1		IMX_GPIO_NR(3, 19)

#define MX6_ARM2_LDB_BACKLIGHT		IMX_GPIO_NR(1, 9)
#define MX6_ARM2_USB_OTG_PWR		IMX_GPIO_NR(3, 22)
#define MX6_ARM2_CAN2_EN		IMX_GPIO_NR(5, 24)
#define MX6_ARM2_SD3_CD			IMX_GPIO_NR(6, 11)
#define MX6_ARM2_SD3_WP			IMX_GPIO_NR(6, 14)
#define MX6_ARM2_CAN1_STBY		IMX_GPIO_NR(7, 12)
#define MX6_ARM2_CAN1_EN		IMX_GPIO_NR(7, 13)

static struct clk *sata_clk;
static int spdif_en;
static int flexcan_en;

extern char *soc_reg_id;
extern char *pu_reg_id;

enum sd_pad_mode {
	SD_PAD_MODE_LOW_SPEED,
	SD_PAD_MODE_MED_SPEED,
	SD_PAD_MODE_HIGH_SPEED,
};

static int plt_sd_pad_change(unsigned int index, int clock)
{
	/* LOW speed is the default state of SD pads */
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;

	iomux_v3_cfg_t *sd_pads_200mhz = NULL;
	iomux_v3_cfg_t *sd_pads_100mhz = NULL;
	iomux_v3_cfg_t *sd_pads_50mhz = NULL;

	u32 sd_pads_200mhz_cnt;
	u32 sd_pads_100mhz_cnt;
	u32 sd_pads_50mhz_cnt;

	switch (index) {
	case 2:
		if (cpu_is_mx6q()) {
			sd_pads_200mhz = cm_fx6_q_sd3_200mhz;
			sd_pads_100mhz = cm_fx6_q_sd3_100mhz;
			sd_pads_50mhz  = cm_fx6_q_sd3_50mhz;

			sd_pads_200mhz_cnt = ARRAY_SIZE(cm_fx6_q_sd3_200mhz);
			sd_pads_100mhz_cnt = ARRAY_SIZE(cm_fx6_q_sd3_100mhz);
			sd_pads_50mhz_cnt  = ARRAY_SIZE(cm_fx6_q_sd3_50mhz);
		} else if (cpu_is_mx6dl()) {
			sd_pads_200mhz = cm_fx6_dl_sd3_200mhz;
			sd_pads_100mhz = cm_fx6_dl_sd3_100mhz;
			sd_pads_50mhz  = cm_fx6_dl_sd3_50mhz;

			sd_pads_200mhz_cnt = ARRAY_SIZE(cm_fx6_dl_sd3_200mhz);
			sd_pads_100mhz_cnt = ARRAY_SIZE(cm_fx6_dl_sd3_100mhz);
			sd_pads_50mhz_cnt  = ARRAY_SIZE(cm_fx6_dl_sd3_50mhz);
		}
		break;
	default:
		printk(KERN_ERR "no such SD host controller index %d\n", index);
		return -EINVAL;
	}

	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;
		BUG_ON(!sd_pads_200mhz);
		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_200mhz,
							sd_pads_200mhz_cnt);
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;
		BUG_ON(!sd_pads_100mhz);
		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_100mhz,
							sd_pads_100mhz_cnt);
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;
		BUG_ON(!sd_pads_50mhz);
		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_50mhz,
							sd_pads_50mhz_cnt);
	}
}

static const struct esdhc_platform_data mx6_arm2_sd3_data __initconst = {
	.cd_gpio		= MX6_ARM2_SD3_CD,
	.wp_gpio		= MX6_ARM2_SD3_WP,
	.support_18v		= 1,
	.support_8bit		= 1,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
	.platform_pad_change	= plt_sd_pad_change,
};

static int gpmi_nand_platform_init(void)
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

static struct gpmi_nand_platform_data mx6_gpmi_nand_platform_data = {
	.platform_init           = gpmi_nand_platform_init,
	.min_prop_delay_in_ns    = 5,
	.max_prop_delay_in_ns    = 9,
	.max_chip_count          = 1,
	.enable_bbt              = 1,
	.enable_ddr              = 0,
};

static int __init board_support_onfi_nand(char *p)
{
	mx6_gpmi_nand_platform_data.enable_ddr = 1;
	return 0;
}
early_param("onfi_support", board_support_onfi_nand);

static const struct anatop_thermal_platform_data cm_fx6_anatop_thermal_data = {
	.name = "anatop_thermal",
};

static const struct imxuart_platform_data mx6_arm2_uart1_data __initconst = {
	.flags      = IMXUART_HAVE_RTSCTS | IMXUART_USE_DCEDTE | IMXUART_SDMA,
	.dma_req_rx = MX6Q_DMA_REQ_UART2_RX,
	.dma_req_tx = MX6Q_DMA_REQ_UART2_TX,
};

static inline void mx6_arm2_init_uart(void)
{
	imx6q_add_imx_uart(3, NULL);
	imx6q_add_imx_uart(1, &mx6_arm2_uart1_data);
}

#define BMCR_PDOWN 0x0800 /* PHY Powerdown */

static int cm_fx6_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

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

static int cm_fx6_spi_cs[] = {
	CM_FX6_ECSPI1_CS0,
	CM_FX6_ECSPI1_CS1,
};

static const struct spi_imx_master cm_fx6_spi_data = {
	.chipselect     = cm_fx6_spi_cs,
	.num_chipselect = ARRAY_SIZE(cm_fx6_spi_cs),
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

static struct spi_board_info cm_fx6_spi0_board_info[] = {
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
	{
		/* The modalias must be the same as spi device driver name */
		.modalias	= "m25p80",
		.max_speed_hz	= 20000000,
		.bus_num	= 0,
		.chip_select	= 0,
		.platform_data	= &cm_fx6_spi_flash_data,
	},
#endif
};

static void cm_fx6_spi_devices_init(void)
{
	spi_register_board_info(cm_fx6_spi0_board_info,
				ARRAY_SIZE(cm_fx6_spi0_board_info));
}

static struct imxi2c_platform_data mx6_arm2_i2c0_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data mx6_arm2_i2c1_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data mx6_arm2_i2c2_data = {
	.bitrate = 400000,
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
};

static struct at24_platform_data cm_fx6_eeprom_pdata = {
        .byte_len       = 256,
        .page_size      = 16,
};

static struct i2c_board_info cm_fx6_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("at24", 0x50),
		.platform_data = &cm_fx6_eeprom_pdata,
	},
};

static void imx6_arm2_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(MX6_ARM2_USB_OTG_PWR, 1);
	else
		gpio_set_value(MX6_ARM2_USB_OTG_PWR, 0);
}

static void __init mx6_arm2_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	/* disable external charger detect,
	 * or it will affect signal quality at dp.
	 */

	ret = gpio_request(MX6_ARM2_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6_ARM2_USB_OTG_PWR:%d\n", ret);
		return;
	}
	gpio_direction_output(MX6_ARM2_USB_OTG_PWR, 0);
	mxc_iomux_set_gpr_register(1, 13, 1, 1);

	mx6_set_otghost_vbus_func(imx6_arm2_usbotg_vbus);
	mx6_usb_dr_init();
}

static struct viv_gpu_platform_data imx6_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_64M,
};

/* HW Initialization, if return 0, initialization is successful. */
static int mx6_arm2_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	/* Enable SATA PWR CTRL_0 of MAX7310 */
//	gpio_request(MX6_ARM2_MAX7310_1_BASE_ADDR, "SATA_PWR_EN");
//	gpio_direction_output(MX6_ARM2_MAX7310_1_BASE_ADDR, 1);

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
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
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;

release_sata_clk:
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);
	/* Disable SATA PWR CTRL_0 of MAX7310 */
//	gpio_request(MX6_ARM2_MAX7310_1_BASE_ADDR, "SATA_PWR_EN");
//	gpio_direction_output(MX6_ARM2_MAX7310_1_BASE_ADDR, 0);

	return ret;
}

static void mx6_arm2_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);

	/* Disable SATA PWR CTRL_0 of MAX7310 */
//	gpio_request(MX6_ARM2_MAX7310_1_BASE_ADDR, "SATA_PWR_EN");
//	gpio_direction_output(MX6_ARM2_MAX7310_1_BASE_ADDR, 0);

}

static struct ahci_platform_data mx6_arm2_sata_data = {
	.init	= mx6_arm2_sata_init,
	.exit	= mx6_arm2_sata_exit,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits	= 4,
	.clk_map_ver	= 2,
};

static struct ipuv3_fb_platform_data sabr_fb_data[] = {
	{ /*fb0*/
	.disp_dev		= "ldb",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB666,
	.mode_str		= "LDB-XGA",
	.default_bpp		= 16,
	.int_clk		= false,
	}, {
	.disp_dev		= "ldb",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB666,
	.mode_str		= "LDB-XGA",
	.default_bpp		= 16,
	.int_clk		= false,
	}, {
	.disp_dev		= "lcd",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB565,
	.mode_str		= "CLAA-WVGA",
	.default_bpp		= 16,
	.int_clk		= false,
	}
};

static void hdmi_init(int ipu_id, int disp_id)
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

/*
 * cm-fx6 shares i2c2 with hdmi ddc, the default is i2c2 function,
 * when hdcp is enabled, the pins should work at ddc function.
 */
static void hdmi_enable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(cm_fx6_dl_hdmi_ddc_pads,
			ARRAY_SIZE(cm_fx6_dl_hdmi_ddc_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(cm_fx6_q_hdmi_ddc_pads,
			ARRAY_SIZE(cm_fx6_q_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(cm_fx6_dl_i2c2_pads,
			ARRAY_SIZE(cm_fx6_dl_i2c2_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(cm_fx6_q_i2c2_pads,
			ARRAY_SIZE(cm_fx6_q_i2c2_pads));
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
	.default_ifmt	= IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id		= 1,
	.disp_id	= 0,
	.ext_ref	= 1,
	.mode		= LDB_SEP0,
	.sec_ipu_id	= 0,
	.sec_disp_id	= 1,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
		.rev		= 4,
		.csi_clk[0]	= "clko_clk",
	}, {
		.rev		= 4,
		.csi_clk[0]	= "clko_clk",
	},
};

static struct platform_pwm_backlight_data mx6_arm2_pwm_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 128,
	.pwm_period_ns	= 50000,
};

static struct gpio mx6_flexcan_gpios[] = {
	{ MX6_ARM2_CAN1_EN, GPIOF_OUT_INIT_LOW, "flexcan1-en" },
	{ MX6_ARM2_CAN1_STBY, GPIOF_OUT_INIT_LOW, "flexcan1-stby" },
	{ MX6_ARM2_CAN2_EN, GPIOF_OUT_INIT_LOW, "flexcan2-en" },
};

static void mx6_flexcan0_switch(int enable)
{
	if (enable) {
//		gpio_set_value(MX6_ARM2_CAN1_EN, 1);
//		gpio_set_value(MX6_ARM2_CAN1_STBY, 1);
	} else {
//		gpio_set_value(MX6_ARM2_CAN1_EN, 0);
//		gpio_set_value(MX6_ARM2_CAN1_STBY, 0);
	}
}

static void mx6_flexcan1_switch(int enable)
{
	if (enable) {
//		gpio_set_value(MX6_ARM2_CAN2_EN, 1);
//		gpio_set_value_cansleep(MX6_ARM2_CAN2_STBY, 1);
	} else {
//		gpio_set_value(MX6_ARM2_CAN2_EN, 0);
//		gpio_set_value_cansleep(MX6_ARM2_CAN2_STBY, 0);
	}
}

static const struct flexcan_platform_data
		mx6_arm2_flexcan_pdata[] __initconst = {
	{
		.transceiver_switch = mx6_flexcan0_switch,
	}, {
		.transceiver_switch = mx6_flexcan1_switch,
	}
};

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

static struct regulator_consumer_supply arm2_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data arm2_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(arm2_vmmc_consumers),
	.consumer_supplies = arm2_vmmc_consumers,
};

static struct fixed_voltage_config arm2_vmmc_reg_config = {
	.supply_name	= "vmmc",
	.microvolts	= 3300000,
	.gpio		= -1,
	.init_data	= &arm2_vmmc_init,
};

static struct platform_device arm2_vmmc_reg_devices = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev		= {
		.platform_data = &arm2_vmmc_reg_config,
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

static int __init early_enable_spdif(char *p)
{
	spdif_en = 1;
	return 0;
}
early_param("spdif", early_enable_spdif);

static int __init early_enable_can(char *p)
{
	flexcan_en = 1;
	return 0;
}
early_param("flexcan", early_enable_can);

static int spdif_clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long rate_actual;
	rate_actual = clk_round_rate(clk, rate);
	clk_set_rate(clk, rate_actual);
	return 0;
}

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx		= 1,		/* enable tx */
	.spdif_rx		= 1,		/* enable rx */
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
	.spdif_clk_set_rate	= spdif_clk_set_rate,
	.spdif_clk		= NULL, /* spdif bus clk */
};

static const struct imx_pcie_platform_data mx6_arm2_pcie_data  __initconst = {
	.pcie_pwr_en	= -EINVAL,
	.pcie_rst	= -EINVAL,
	.pcie_wake_up	= -EINVAL,
	.pcie_dis	= -EINVAL,
};

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

/*
 * Board specific initialization.
 */
static void __init cm_fx6_init(void)
{
	int i;
	int ret;

	iomux_v3_cfg_t *common_pads = NULL;
	iomux_v3_cfg_t *spdif_pads = NULL;
	iomux_v3_cfg_t *flexcan_pads = NULL;

	int common_pads_cnt;
	int spdif_pads_cnt;
	int flexcan_pads_cnt;


	/*
	 * common pads: pads are non-shared with others on this board
	 * feature_pds: pads are shared with others on this board
	 */

	if (cpu_is_mx6q()) {
		common_pads = cm_fx6_q_common_pads;
		spdif_pads = cm_fx6_q_spdif_pads;
		flexcan_pads = cm_fx6_q_can_pads;

		common_pads_cnt = ARRAY_SIZE(cm_fx6_q_common_pads);
		spdif_pads_cnt =  ARRAY_SIZE(cm_fx6_q_spdif_pads);
		flexcan_pads_cnt = ARRAY_SIZE(cm_fx6_q_can_pads);
	} else if (cpu_is_mx6dl()) {
		common_pads = cm_fx6_dl_pads;
		spdif_pads = cm_fx6_dl_spdif_pads;
		flexcan_pads = cm_fx6_dl_can_pads;

		common_pads_cnt = ARRAY_SIZE(cm_fx6_dl_pads);
		spdif_pads_cnt =  ARRAY_SIZE(cm_fx6_dl_spdif_pads);
		flexcan_pads_cnt = ARRAY_SIZE(cm_fx6_dl_can_pads);
	}

	BUG_ON(!common_pads);
	mxc_iomux_v3_setup_multiple_pads(common_pads, common_pads_cnt);

	if (spdif_en) {
		BUG_ON(!spdif_pads);
		mxc_iomux_v3_setup_multiple_pads(spdif_pads, spdif_pads_cnt);
	}

	/*
	 * S/PDIF out and can1 stby are mutually exclusive because both
	 * use GPIO_17.
	 */
	if (!spdif_en && flexcan_en) {
		BUG_ON(!flexcan_pads);
		mxc_iomux_v3_setup_multiple_pads(flexcan_pads,
						flexcan_pads_cnt);
	}

	/*
	 * the following is the common devices support on the shared ARM2 boards
	 * Since i.MX6DQ/DL share the same memory/Register layout, we don't
	 * need to diff the i.MX6DQ or i.MX6DL here. We can simply use the
	 * mx6q_add_features() for the shared devices. For which only exist
	 * on each indivual SOC, we can use cpu_is_mx6q/6dl() to diff it.
	 */
	gp_reg_id = arm2_dvfscore_data.reg_id;
	soc_reg_id = arm2_dvfscore_data.soc_id;
	pu_reg_id = arm2_dvfscore_data.pu_id;
	mx6_arm2_init_uart();

	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q())
		imx6q_add_ipuv3(1, &ipu_data[1]);

	if (cpu_is_mx6dl()) {
		ldb_data.ipu_id = 0;
		ldb_data.disp_id = 0;
		for (i = 0; i < ARRAY_SIZE(sabr_fb_data) / 2; i++)
			imx6q_add_ipuv3fb(i, &sabr_fb_data[i]);
	} else {
		for (i = 0; i < ARRAY_SIZE(sabr_fb_data); i++)
			imx6q_add_ipuv3fb(i, &sabr_fb_data[i]);
	}

	imx6q_add_vdoa();
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);

	imx6q_add_imx_snvs_rtc();

	imx6q_add_imx_caam();

	imx6q_add_imx_i2c(0, &mx6_arm2_i2c0_data);
	imx6q_add_imx_i2c(1, &mx6_arm2_i2c1_data);
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	if (!spdif_en) {
		imx6q_add_imx_i2c(2, &mx6_arm2_i2c2_data);
		i2c_register_board_info(2, cm_fx6_i2c2_board_info,
				ARRAY_SIZE(cm_fx6_i2c2_board_info));
	}

	cm_fx6_init_led();

	/* SPI */
	imx6q_add_ecspi(0, &cm_fx6_spi_data);
	cm_fx6_spi_devices_init();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &cm_fx6_anatop_thermal_data);

	imx6_init_fec(cm_fx6_fec_data);

	imx6q_add_pm_imx(0, &mx6_arm2_pm_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6_arm2_sd3_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6_gpu_pdata);

	if (cpu_is_mx6q())
		imx6q_add_ahci(0, &mx6_arm2_sata_data);

	imx6q_add_vpu();
	mx6_arm2_init_usb();
	platform_device_register(&arm2_vmmc_reg_devices);
	mx6_cpu_regulator_init();

	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	gpio_request_one(MX6_ARM2_LDB_BACKLIGHT, GPIOF_OUT_INIT_HIGH,
			 "ldb-backlight");
	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();
	imx6q_add_gpmi(&mx6_gpmi_nand_platform_data);

	imx6q_add_dvfs_core(&arm2_dvfscore_data);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm_backlight(0, &mx6_arm2_pwm_backlight_data);

	if (spdif_en) {
		mxc_spdif_data.spdif_core_clk = clk_get_sys("mxc_spdif.0", NULL);
		clk_put(mxc_spdif_data.spdif_core_clk);
		imx6q_add_spdif(&mxc_spdif_data);
		imx6q_add_spdif_dai();
		imx6q_add_spdif_audio_device();
	} else if (flexcan_en) {
		ret = gpio_request_array(mx6_flexcan_gpios,
				ARRAY_SIZE(mx6_flexcan_gpios));
		if (ret) {
			pr_err("failed to request flexcan-gpios: %d\n", ret);
		} else {
			imx6q_add_flexcan0(&mx6_arm2_flexcan_pdata[0]);
			imx6q_add_flexcan1(&mx6_arm2_flexcan_pdata[1]);
		}
	}

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
	imx6q_add_mlb150(&mx6_arm2_mlb150_data);

	/* Add PCIe RC interface support */
	imx6q_add_pcie(&mx6_arm2_pcie_data);
	imx6q_add_busfreq();
}

extern void __iomem *twd_base;
static void __init cm_fx6_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
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
