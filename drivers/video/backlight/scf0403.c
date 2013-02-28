/*
 * scf0403.c -- support for DataImage SCF0403852GGU04 LCD
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

#define param(x) ((x) | 0x100)

struct scf0403_priv {
	struct spi_device *spi;
	struct lcd_device *ld;
	unsigned int reset_gpio;
	int lcd_state;
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

static int scf0403_spi_transfer(struct spi_device *spi, int cmd, const u8 *wbuf,
			   int wlen, u8 *rbuf, int rlen)
{
	struct spi_message	m;
	struct spi_transfer	*x, xfer[4];
	u16			w;
	int			r = 0;

	spi_message_init(&m);

	memset(xfer, 0, sizeof(xfer));
	x = &xfer[0];

	cmd &=  0xff;
	x->tx_buf		= &cmd;
	x->bits_per_word	= 9;
	x->len			= 2;
	spi_message_add_tail(x, &m);

	if (wlen) {
		x++;
		x->tx_buf		= wbuf;
		x->len			= wlen;
		x->bits_per_word	= 9;
		spi_message_add_tail(x, &m);
	}

	if (rlen) {
		x++;
		x->rx_buf	= &w;
		x->len		= 1;
		spi_message_add_tail(x, &m);

		if (rlen > 1) {
			/*
			 * Arrange for the extra clock before the first
			 * data bit.
			 */
			x->bits_per_word = 9;
			x->len		 = 2;

			x++;
			x->rx_buf	 = &rbuf[1];
			x->len		 = rlen - 1;
			spi_message_add_tail(x, &m);
		}
	}

	r = spi_sync(spi, &m);

	if (rlen)
		rbuf[0] = w & 0xff;

	return r;
}

static void scf0403_lcd_init(struct spi_device *spi)
{
	struct scf0403_pdata *pdata = spi->dev.platform_data;

	const u8 memAccessCtrl = 0x36;
	const u16 memAccessCtrlParam[] = { param(0x08) };

	const u8 ifPixelFormat = 0x3A;
	const u16 ifPixelFormatParam[] = { param(0x66) };

	const u8 ifModeCtrl = 0xB0;
	const u16 ifModeCtrlParam[] = { param(0x01) };

	const u8 dispFuncCtrl = 0xB6;
	const u16 dispFuncCtrlParam[] = {
			param(0x22), param(0xe2), param(0xFF), param(0x04)
	};

	const u8 vcomCtrl = 0xC5;
	const u16 vcomCtrlParam[] = { param(0x00), param(0x6A) };

	const u8 gamma = 0xE0;
	const u16 gammaParam[] = {
			param(0x00), param(0x07), param(0x0d), param(0x10),
			param(0x13), param(0x19), param(0x0f), param(0x0c),
			param(0x05), param(0x08), param(0x06), param(0x13),
			param(0x0f), param(0x30), param(0x20), param(0x1f)
	};

	const u8 negGamma = 0xE1;
	const u16 negGammaParam[] = {
			param(0x1F), param(0x20), param(0x30), param(0x0F),
			param(0x13), param(0x06), param(0x08), param(0x05),
			param(0x0C), param(0x0F), param(0x19), param(0x13),
			param(0x10), param(0x0D), param(0x07), param(0x00)
	};

	const u8 dispInvCtrl = 0xB4;
	const u16 dispInvCtrlParam[] = { 0x02 };

	/* reset LCD */
	scf0403_gpio_reset(pdata->reset_gpio);

	/* perform initialization sequence */
	if (scf0403_spi_transfer(spi, dispFuncCtrl,
				(const u8 *)dispFuncCtrlParam,
				sizeof(dispFuncCtrlParam), NULL, 0) < 0) {
		dev_err(&spi->dev, "Setting Display Function Control - failed");
	}

	if (scf0403_spi_transfer(spi, vcomCtrl, (const u8 *)vcomCtrlParam,
				sizeof(vcomCtrlParam), NULL, 0) < 0) {
		dev_err(&spi->dev, "Setting VCOM Control - failed");
	}

	if (scf0403_spi_transfer(spi, memAccessCtrl,
				(const u8 *)memAccessCtrlParam,
				sizeof(memAccessCtrlParam), NULL, 0) < 0) {
		dev_err(&spi->dev, "Setting Memory Access Control - failed");
	}

	if (scf0403_spi_transfer(spi, ifPixelFormat,
				(const u8 *)ifPixelFormatParam,
				sizeof(ifPixelFormatParam), NULL, 0) < 0) {
		dev_err(&spi->dev, "Setting Interface Pixel Format - failed");
	}

	if (scf0403_spi_transfer(spi, gamma,
				(const u8 *)gammaParam,
				sizeof(gammaParam), NULL, 0) < 0) {
		dev_err(&spi->dev, "Setting Gamma - failed");
	}

	if (scf0403_spi_transfer(spi, negGamma, (const u8 *)negGammaParam,
				sizeof(negGammaParam), NULL, 0) < 0) {
		dev_err(&spi->dev, "Setting Negative Gamma - failed");
	}

	msleep(20);

	if (scf0403_spi_transfer(spi, ifModeCtrl, (const u8 *)ifModeCtrlParam,
				sizeof(ifModeCtrlParam), NULL, 0) < 0) {
		dev_err(&spi->dev, "Setting Interface Mode Control - failed");
	}

	if (scf0403_spi_transfer(spi, dispInvCtrl, (const u8 *)dispInvCtrlParam,
				sizeof(dispInvCtrlParam), NULL, 0) < 0) {
		dev_err(&spi->dev, "Setting Dislay Inversion Control - failed");
	}

	msleep(100);
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

	const u16 slpout = 0x11;
	const u16 dison = 0x29;

	const u16 slpin = 0x10;
	const u16 disoff = 0x28;

	dev_dbg(&spi->dev, "LCD Power set, power = %d, curr power = %d\n",
		power, priv->lcd_state);

	if (power <= FB_BLANK_NORMAL) {
		if (priv->lcd_state <= FB_BLANK_NORMAL) {
			dev_dbg(&spi->dev, "LCD sleep in-out\n");
		} else if (priv->lcd_state < FB_BLANK_POWERDOWN) {
			dev_dbg(&spi->dev, "Resuming LCD\n");
			scf0403_spi_transfer(spi, dison, NULL, 0, NULL, 0);
			msleep(100);
			scf0403_spi_transfer(spi, slpout, NULL, 0, NULL, 0);
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

			scf0403_spi_transfer(spi, disoff, NULL, 0, NULL, 0);
			msleep(60);
			scf0403_spi_transfer(spi, slpin, NULL, 0, NULL, 0);
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
			scf0403_spi_transfer(spi, disoff, NULL, 0, NULL, 0);
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
	const u8 disp_ids = 0x04;
	u8 ids_buff[3] = { 0x00 };
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

	if (scf0403_spi_transfer(spi, disp_ids, NULL, 0, ids_buff, 3) < 0 ||
	    !(ids_buff[0] || ids_buff[1] || ids_buff[2])) {
		dev_err(&spi->dev, "IDs read failed\n");
		ret = -ENODEV;
		goto free_gpio;
	}

	dev_info(&spi->dev, "Device IDs: 0x%02X 0x%02X 0x%02X\n",
			 ids_buff[0], ids_buff[1], ids_buff[2]);

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
