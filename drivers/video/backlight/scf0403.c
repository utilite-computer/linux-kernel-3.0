/*
 * scf0403.c -- support for DataImage SCF0403 LCD
 *
 * Copyright (c) 2012 Anders Electronics plc. All Rights Reserved.
 * Copyright (c) 2012 CompuLab, Ltd
 *           Dmitry Lifshitz <lifshitz@compulab.co.il>
 *           Ilya Ledvich <ilya@compulab.co.il>
 *
 *  Inspired by Alberto Panizzo <maramaopercheseimorto@gmail.com> &
 *	Marek Vasut work in l4f00242t03.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/lcd.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/spi/scf0403.h>

struct scf0403_cmd {
	u16 cmd;
	u16 *params;
	int count;
};

struct scf0403_initseq_entry {
	struct scf0403_cmd cmd;
	int delay_ms;
};

struct scf0403_priv {
	struct spi_device *spi;
	struct lcd_device *ld;
	unsigned int reset_gpio;
	int lcd_state;
	u32 rddid;
	struct scf0403_initseq_entry *init_seq;
	int seq_size;
};

#define SCF0403852GGU04_ID 0x000080

/* SCF0403526GGU20 model commands parameters */
static u16 extcmd_params_sn20[]		= {0xff, 0x98, 0x06};
static u16 spiinttype_params_sn20[]	= {0x60};
static u16 bc_params_sn20[]		= {
		0x01, 0x10, 0x61, 0x74, 0x01, 0x01, 0x1B,
		0x12, 0x71, 0x00, 0x00, 0x00, 0x01, 0x01,
		0x05, 0x00, 0xFF, 0xF2, 0x01, 0x00, 0x40,
};
static u16 bd_params_sn20[] = {0x01, 0x23, 0x45, 0x67, 0x01, 0x23, 0x45, 0x67};
static u16 be_params_sn20[] = {
		0x01, 0x22, 0x22, 0xBA, 0xDC, 0x26, 0x28, 0x22,	0x22,
};
static u16 vcom_params_sn20[]		= {0x74};
static u16 vmesur_params_sn20[]		= {0x7F, 0x0F, 0x00};
static u16 powerctl_params_sn20[]	= {0x03, 0x0b, 0x00};
static u16 lvglvolt_params_sn20[]	= {0x08};
static u16 engsetting_params_sn20[]	= {0x00, 0x00, 0x00, 0x00, 0x00, 0x20};
static u16 dispfunc_params_sn20[]	= {0xa0};
static u16 dvddvolt_params_sn20[]	= {0x74};
static u16 dispinv_params_sn20[]	= {0x00, 0x00, 0x00};
static u16 panelres_params_sn20[]	= {0x82};
static u16 framerate_params_sn20[]	= {0x00, 0x13, 0x13};
static u16 timing_params_sn20[]		= {0x80, 0x05, 0x40, 0x28};
static u16 powerctl2_params_sn20[]	= {0x17, 0x75, 0x79, 0x20};
static u16 memaccess_params_sn20[]	= {0x00};
static u16 pixfmt_params_sn20[]		= {0x66};
static u16 pgamma_params_sn20[]		= {
		0x00, 0x03, 0x0b, 0x0c, 0x0e, 0x08, 0xc5, 0x04,
		0x08, 0x0c, 0x13, 0x11, 0x11, 0x14, 0x0c, 0x10,
};
static u16 ngamma_params_sn20[] = {
		0x00, 0x0d, 0x11, 0x0c, 0x0c, 0x04, 0x76, 0x03,
		0x08, 0x0b, 0x16, 0x10, 0x0d, 0x16, 0x0a, 0x00,
};
static u16 tearing_params_sn20[] = {0x00};

/* SCF0403852GGU04 model commands parameters */
static u16 memaccess_params_sn04[]	= {0x08};
static u16 pixfmt_params_sn04[]		= {0x66};
static u16 modectl_params_sn04[]	= {0x01};
static u16 dispfunc_params_sn04[]	= {0x22, 0xe2, 0xFF, 0x04};
static u16 vcom_params_sn04[]		= {0x00, 0x6A};
static u16 pgamma_params_sn04[]		= {
		0x00, 0x07, 0x0d, 0x10, 0x13, 0x19, 0x0f, 0x0c,
		0x05, 0x08, 0x06, 0x13,	0x0f, 0x30, 0x20, 0x1f,
};
static u16 ngamma_params_sn04[]		= {
		0x1F, 0x20, 0x30, 0x0F, 0x13, 0x06, 0x08, 0x05,
		0x0C, 0x0F, 0x19, 0x13, 0x10, 0x0D, 0x07, 0x00,
};
static u16 dispinv_params_sn04[]	= {0x02};

/* Common commands */
static struct scf0403_cmd scf0403_cmd_slpout	= {0x11, NULL, 0};
static struct scf0403_cmd scf0403_cmd_dison	= {0x29, NULL, 0};
static struct scf0403_cmd scf0403_cmd_slpin	= {0x10, NULL, 0};
static struct scf0403_cmd scf0403_cmd_disoff	= {0x28, NULL, 0};

/* SCF0403852GGU04 init sequence */
static struct scf0403_initseq_entry scf0403_initseq_entry_sn04[] = {
	{{0x36, memaccess_params_sn04,	ARRAY_SIZE(memaccess_params_sn04)}, 0},
	{{0x3A, pixfmt_params_sn04,	ARRAY_SIZE(pixfmt_params_sn04)}, 0},
	{{0xB6, dispfunc_params_sn04,	ARRAY_SIZE(dispfunc_params_sn04)}, 0},
	{{0xC5, vcom_params_sn04,	ARRAY_SIZE(vcom_params_sn04)}, 0},
	{{0xE0, pgamma_params_sn04,	ARRAY_SIZE(pgamma_params_sn04)}, 0},
	{{0xE1, ngamma_params_sn04,	ARRAY_SIZE(ngamma_params_sn04)}, 20},
	{{0xB0, modectl_params_sn04,	ARRAY_SIZE(modectl_params_sn04)}, 0},
	{{0xB4, dispinv_params_sn04,	ARRAY_SIZE(dispinv_params_sn04)}, 100},
};

/* SCF0403526GGU20 init sequence */
static struct scf0403_initseq_entry scf0403_initseq_entry_sn20[] = {
	{{0xff, extcmd_params_sn20,	ARRAY_SIZE(extcmd_params_sn20)}, 0},
	{{0xba, spiinttype_params_sn20,	ARRAY_SIZE(spiinttype_params_sn20)}, 0},
	{{0xbc, bc_params_sn20,		ARRAY_SIZE(bc_params_sn20)}, 0},
	{{0xbd, bd_params_sn20,		ARRAY_SIZE(bd_params_sn20)}, 0},
	{{0xbe, be_params_sn20,		ARRAY_SIZE(be_params_sn20)}, 0},
	{{0xc7, vcom_params_sn20,	ARRAY_SIZE(vcom_params_sn20)}, 0},
	{{0xed, vmesur_params_sn20,	ARRAY_SIZE(vmesur_params_sn20)}, 0},
	{{0xc0, powerctl_params_sn20,	ARRAY_SIZE(powerctl_params_sn20)}, 0},
	{{0xfc, lvglvolt_params_sn20,	ARRAY_SIZE(lvglvolt_params_sn20)}, 0},
	{{0xb6, dispfunc_params_sn20,	ARRAY_SIZE(dispfunc_params_sn20)}, 0},
	{{0xdf, engsetting_params_sn20,	ARRAY_SIZE(engsetting_params_sn20)}, 0},
	{{0xf3, dvddvolt_params_sn20,	ARRAY_SIZE(dvddvolt_params_sn20)}, 0},
	{{0xb4, dispinv_params_sn20,	ARRAY_SIZE(dispinv_params_sn20)}, 0},
	{{0xf7, panelres_params_sn20,	ARRAY_SIZE(panelres_params_sn20)}, 0},
	{{0xb1, framerate_params_sn20,	ARRAY_SIZE(framerate_params_sn20)}, 0},
	{{0xf2, timing_params_sn20,	ARRAY_SIZE(timing_params_sn20)}, 0},
	{{0xc1, powerctl2_params_sn20,	ARRAY_SIZE(powerctl2_params_sn20)}, 0},
	{{0x36, memaccess_params_sn20,	ARRAY_SIZE(memaccess_params_sn20)}, 0},
	{{0x3a, pixfmt_params_sn20,	ARRAY_SIZE(pixfmt_params_sn20)}, 0},
	{{0xe0, pgamma_params_sn20,	ARRAY_SIZE(pgamma_params_sn20)}, 0},
	{{0xe1, ngamma_params_sn20,	ARRAY_SIZE(ngamma_params_sn20)}, 0},
	{{0x35, tearing_params_sn20,	ARRAY_SIZE(tearing_params_sn20)}, 0},
};

static void scf0403_gpio_reset(unsigned int gpio)
{
	if (!gpio_is_valid(gpio))
		return;

	gpio_set_value_cansleep(gpio, 1);
	msleep(100);
	gpio_set_value_cansleep(gpio, 0);
	msleep(40);
	gpio_set_value_cansleep(gpio, 1);
	msleep(100);
}

static int scf0403_spi_write(struct spi_device *spi, u16 buf)
{
	struct spi_message  m;
	struct spi_transfer x;
	int ret = 0;

	memset(&x, 0, sizeof(x));
	spi_message_init(&m);

	x.tx_buf	= &buf;
	x.len		= 2;
	x.bits_per_word = 9;

	spi_message_add_tail(&x, &m);
	ret = spi_sync(spi, &m);

	return ret;
}

static int scf0403_spi_read_rddid(struct spi_device *spi, u32 *rddid)
{
	struct spi_message m;
	struct spi_transfer *x, xfer[4];
	u32 ids_buf = 0x00;
	u32 dummy_buf = 0x00;
	u16 cmd = 0x04;
	int ret = 0;

	memset(xfer, 0, sizeof(xfer));
	spi_message_init(&m);

	/* Here 9 bits required to transmit a command */
	x = &xfer[0];
	x->tx_buf = &cmd;
	x->len = 2;
	x->bits_per_word = 9;
	spi_message_add_tail(x, &m);

	/*
	 * Here 8 + 1 bits required to arrange extra clock cycle
	 * before the first data bit.
	 * According to the datasheet - first parameter is the dummy data.
	 */
	x++;
	x->rx_buf = &dummy_buf;
	x->len = 2;
	x->bits_per_word = 9;
	spi_message_add_tail(x, &m);

	/* Read rest of the data */
	x++;
	x->rx_buf = &ids_buf;
	x->len = 3;
	x->bits_per_word = 8;
	spi_message_add_tail(x, &m);

	ret = spi_sync(spi, &m);

	if (ret == 0)
		*rddid = ids_buf;

	return ret;
}

static int scf0403_spi_transfer(struct spi_device *spi, struct scf0403_cmd *cmd)
{
	int i;
	int ret = 0;

	ret = scf0403_spi_write(spi, cmd->cmd);
	if (ret < 0)
		return ret;

	for (i = 0; i < cmd->count; i++) {
		ret = scf0403_spi_write(spi, (cmd->params[i] | 0x100));
		if (ret < 0)
			return ret;
	}

	return ret;
}

static void scf0403_lcd_init(struct spi_device *spi)
{
	struct scf0403_pdata *pdata = spi->dev.platform_data;
	struct scf0403_priv *priv = dev_get_drvdata(&spi->dev);
	int i;

	dev_dbg(&spi->dev, "initializing LCD\n");

	/* reset LCD */
	scf0403_gpio_reset(pdata->reset_gpio);

	for (i = 0; i < priv->seq_size; i++) {
		if (scf0403_spi_transfer(spi, &priv->init_seq[i].cmd) < 0)
			dev_err(&spi->dev, "SPI transfer - failed");

		msleep(priv->init_seq[i].delay_ms);
	}
}

static int scf0403_lcd_power_get(struct lcd_device *ld)
{
	struct scf0403_priv *priv = lcd_get_data(ld);

	return priv->lcd_state;
}

static int scf0403_lcd_power_set(struct lcd_device *ld, int power)
{
	struct scf0403_priv *priv = lcd_get_data(ld);
	struct spi_device *spi = priv->spi;

	dev_dbg(&spi->dev, "LCD Power set, power = %d, curr power = %d\n",
		power, priv->lcd_state);

	if (power <= FB_BLANK_NORMAL) {
		if (priv->lcd_state <= FB_BLANK_NORMAL) {
			dev_dbg(&spi->dev, "LCD sleep in-out\n");
		} else if (priv->lcd_state < FB_BLANK_POWERDOWN) {
			dev_dbg(&spi->dev, "Resuming LCD\n");

			scf0403_spi_transfer(spi, &scf0403_cmd_dison);
			msleep(100);
			scf0403_spi_transfer(spi, &scf0403_cmd_slpout);
		} else {
			/* priv->lcd_state == FB_BLANK_POWERDOWN */
			scf0403_lcd_init(spi);
			priv->lcd_state = FB_BLANK_VSYNC_SUSPEND;
			scf0403_lcd_power_set(priv->ld, power);
		}

	} else if (power < FB_BLANK_POWERDOWN) {
		if (priv->lcd_state <= FB_BLANK_NORMAL) {
			/* Send the display in standby */
			dev_dbg(&spi->dev, "Standby the LCD\n");

			scf0403_spi_transfer(spi, &scf0403_cmd_disoff);
			msleep(60);
			scf0403_spi_transfer(spi, &scf0403_cmd_slpin);
		} else if (priv->lcd_state < FB_BLANK_POWERDOWN) {
			/* Do nothing, the LCD is already in standby */
		} else {
			/* priv->lcd_state == FB_BLANK_POWERDOWN */
			scf0403_lcd_init(spi);
			priv->lcd_state = FB_BLANK_UNBLANK;
			scf0403_lcd_power_set(ld, power);
		}
	} else {
		/* power == FB_BLANK_POWERDOWN */
		if (priv->lcd_state != FB_BLANK_POWERDOWN) {
			/* Clear the screen before shutting down */
			scf0403_spi_transfer(spi, &scf0403_cmd_disoff);
			msleep(60);
		}
	}

	priv->lcd_state = power;

	return 0;
}

static struct lcd_ops l4f_ops = {
	.set_power	= scf0403_lcd_power_set,
	.get_power	= scf0403_lcd_power_get,
};

static int __devinit scf0403_probe(struct spi_device *spi)
{
	struct scf0403_priv *priv;
	struct scf0403_pdata *pdata = spi->dev.platform_data;
	int ret, reset_gpio = -EINVAL;

	dev_dbg(&spi->dev, "LCD probe.\n");

	priv = kzalloc(sizeof(struct scf0403_priv), GFP_KERNEL);
	if (priv == NULL) {
		dev_err(&spi->dev, "No memory for this device.\n");
		return -ENOMEM;
	}

	if (pdata && gpio_is_valid(pdata->reset_gpio)) {
		reset_gpio = pdata->reset_gpio;
		ret = gpio_request_one(reset_gpio, GPIOF_OUT_INIT_LOW,
				       "lcd reset");
		if (ret) {
			dev_err(&spi->dev,
				"Failed requesting reset GPIO%d: %d\n",
				reset_gpio, ret);
			goto err;
		}

		gpio_export(reset_gpio, 0);
	}

	priv->reset_gpio = reset_gpio;
	dev_set_drvdata(&spi->dev, priv);
	spi->bits_per_word = 8;
	spi_setup(spi);

	priv->spi = spi;

	/* reset LCD */
	scf0403_gpio_reset(priv->reset_gpio);

	ret = scf0403_spi_read_rddid(spi, &priv->rddid);
	if (ret < 0) {
		dev_err(&spi->dev, "IDs read failed\n");
		goto free_gpio;
	}

	dev_info(&spi->dev, "Device ID: 0x%06X\n", priv->rddid);

	if (priv->rddid == SCF0403852GGU04_ID) {
		priv->init_seq = scf0403_initseq_entry_sn04;
		priv->seq_size = ARRAY_SIZE(scf0403_initseq_entry_sn04);
	} else {
		priv->init_seq = scf0403_initseq_entry_sn20;
		priv->seq_size = ARRAY_SIZE(scf0403_initseq_entry_sn20);
	}

	priv->ld = lcd_device_register("scf0403", &spi->dev, priv, &l4f_ops);
	if (IS_ERR(priv->ld)) {
		ret = PTR_ERR(priv->ld);
		goto free_gpio;
	}

	/* Init the LCD */
	scf0403_lcd_init(spi);

	priv->lcd_state = FB_BLANK_VSYNC_SUSPEND;
	scf0403_lcd_power_set(priv->ld, FB_BLANK_UNBLANK);

	dev_dbg(&spi->dev, "DataImage scf0403 lcd probed.\n");

	return 0;


free_gpio:
	gpio_free(priv->reset_gpio);
err:
	dev_set_drvdata(&spi->dev, NULL);
	kfree(priv);

	return ret;
}

static int __devexit scf0403_remove(struct spi_device *spi)
{
	struct scf0403_priv *priv = dev_get_drvdata(&spi->dev);

	scf0403_lcd_power_set(priv->ld, FB_BLANK_POWERDOWN);
	lcd_device_unregister(priv->ld);

	dev_set_drvdata(&spi->dev, NULL);
	gpio_free(priv->reset_gpio);
	kfree(priv);

	return 0;
}

#ifdef CONFIG_PM
static int scf0403_suspend(struct spi_device *spi, pm_message_t state)
{
	struct scf0403_priv *priv = dev_get_drvdata(&spi->dev);

	return scf0403_lcd_power_set(priv->ld, FB_BLANK_POWERDOWN);
}

static int scf0403_resume(struct spi_device *spi)
{
	struct scf0403_priv *priv = dev_get_drvdata(&spi->dev);

	return scf0403_lcd_power_set(priv->ld, FB_BLANK_UNBLANK);
}
#else
#define scf0403_suspend  NULL
#define scf0403_resume   NULL
#endif

static void scf0403_shutdown(struct spi_device *spi)
{
	struct scf0403_priv *priv = dev_get_drvdata(&spi->dev);

	if (priv)
		scf0403_lcd_power_set(priv->ld, FB_BLANK_POWERDOWN);

}

static struct spi_driver scf0403_driver = {
	.driver = {
		.name	= "scf0403",
		.owner	= THIS_MODULE,
	},
	.probe		= scf0403_probe,
	.remove		= __devexit_p(scf0403_remove),
	.shutdown	= scf0403_shutdown,
	.suspend        = scf0403_suspend,
	.resume         = scf0403_resume,
};

static __init int scf0403_init(void)
{
	return spi_register_driver(&scf0403_driver);
}

static __exit void scf0403_exit(void)
{
	spi_unregister_driver(&scf0403_driver);
}

module_init(scf0403_init);
module_exit(scf0403_exit);

MODULE_DESCRIPTION("DataImage scf0403 LCD");
MODULE_LICENSE("GPL v2");
