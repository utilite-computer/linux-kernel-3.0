/*
 * An rtc/i2c driver for the EM Microelectronic EM3027
 * Copyright 2011 CompuLab, Ltd.
 *
 * Author: Mike Rapoport <mike@compulab.co.il>
 *
 * Based on rtc-ds1672.c by Alessandro Zummo <a.zummo@towertech.it>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/rtc/rtc-em3027.h>

/* Registers */
#define EM3027_REG_ON_OFF_CTRL		0x00
#define EM3027_REG_IRQ_CTRL		0x01
#define EM3027_REG_IRQ_FLAGS		0x02
#define EM3027_REG_STATUS		0x03
#define EM3027_REG_RST_CTRL		0x04

#define EM3027_REG_WATCH_SEC		0x08
#define EM3027_REG_WATCH_MIN		0x09
#define EM3027_REG_WATCH_HOUR		0x0a
#define EM3027_REG_WATCH_DATE		0x0b
#define EM3027_REG_WATCH_DAY		0x0c
#define EM3027_REG_WATCH_MON		0x0d
#define EM3027_REG_WATCH_YEAR		0x0e

#define EM3027_REG_ALARM_SEC		0x10
#define EM3027_REG_ALARM_MIN		0x11
#define EM3027_REG_ALARM_HOUR		0x12
#define EM3027_REG_ALARM_DATE		0x13
#define EM3027_REG_ALARM_DAY		0x14
#define EM3027_REG_ALARM_MON		0x15
#define EM3027_REG_ALARM_YEAR		0x16

#define EM3027_REG_EEPROM_CTRL		0x30

/* Bits in registers */
#define EM3027_BIT_ON_OFF_CLK_nINT	0x80
#define EM3027_BIT_ON_OFF_TD1		0x40
#define EM3027_BIT_ON_OFF_TD0		0x20
#define EM3027_BIT_ON_OFF_SR		0x10
#define EM3027_BIT_ON_OFF_EE_REFRESH	0x08
#define EM3027_BIT_ON_OFF_TMR_AUTOREL	0x04
#define EM3027_BIT_ON_OFF_TMR_ENABLE	0x02
#define EM3027_BIT_ON_OFF_WATCH_1HZ	0x01

#define EM3027_BIT_IRQ_SR		0x10
#define EM3027_BIT_IRQ_VLOW2		0x08
#define EM3027_BIT_IRQ_VLOW1		0x04
#define EM3027_BIT_IRQ_TIMER		0x02
#define EM3027_BIT_IRQ_ALARM		0x01

#define EM3027_IRQ_MASK			0x1f
#define EM3027_TRICKLE_CHARGER_MASK	0xf0

static struct i2c_driver em3027_driver;


/**
 * Read one or more consequent registers, starting with 'addr'
 *
 * The starting address is written to the device,
 * then the register values are read back.
 */
static int em3027_read_register(struct i2c_client *client,
				unsigned char addr,
				unsigned char *buf,
				size_t count)
{
	struct i2c_msg msgs[] = {
		{client->addr, 0, 1, &addr},
		{client->addr, I2C_M_RD, count, buf},
	};

	if (i2c_transfer(client->adapter, &msgs[0], 2) != 2) {
		dev_err(&client->dev, "%s: could not read register %02x \n",
			__func__, addr);
		return -EIO;
	}

	return 0;
}

/**
 * Write one or more registers, starting with 'addr'
 *
 * The starting address is written first,
 * then the register values.
 * Since the address is 'buf[0]', 'count' is greater by 1
 * than it would be in reading the same data set.
 */
static int em3027_write_register(struct i2c_client *client,
				 unsigned char *buf,
				 size_t count)
{
	struct i2c_msg msg = {client->addr, 0, count, buf};

	if (i2c_transfer(client->adapter, &msg, 1) != 1) {
		dev_err(&client->dev, "%s: could not write register %02x \n",
			__func__, buf[0]);
		return -EIO;
	}

	return 0;
}

/**
 * Update particular bits in a register
 */
static int em3027_update_register(struct i2c_client *client,
				  unsigned char addr,
				  unsigned char mask,
				  unsigned char value)
{
	int err;
	unsigned char old_value;
	unsigned char buf[2];

	err = em3027_read_register(client, addr, &old_value, 1);
	if (err)
		return err;

	buf[0] = addr;
	buf[1] = (old_value & ~mask) | (value & mask);

	return em3027_write_register(client, buf, ARRAY_SIZE(buf));
}

/**
 * Update particular bits in a EEPROM backed register
 * Do not burn EEPROM if unnecessary
 */
static int em3027_update_eeprom_register(struct i2c_client *client,
					 unsigned char addr,
					 unsigned char mask,
					 unsigned char value)
{
	int err;
	unsigned char old_value;
	unsigned char buf[2];

	err = em3027_read_register(client, addr, &old_value, 1);
	if (err)
		return err;

	buf[0] = addr;
	buf[1] = (old_value & ~mask) | (value & mask);
	if (old_value != buf[1]) {
		dev_dbg(&client->dev, "%s: [%02x] %02x -> %02x \n", __func__,
			addr, old_value, buf[1]);
		err = em3027_write_register(client, buf, ARRAY_SIZE(buf));
	}

	return err;
}

static int em3027_get_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char buf[7];
	int err;

	err = em3027_read_register(client, EM3027_REG_WATCH_SEC, buf, 7);
	if (err)
		return err;

	tm->tm_sec	= bcd2bin(buf[0]);
	tm->tm_min	= bcd2bin(buf[1]);
	tm->tm_hour	= bcd2bin(buf[2]);
	tm->tm_mday	= bcd2bin(buf[3]);
	tm->tm_wday	= bcd2bin(buf[4]);
	tm->tm_mon	= bcd2bin(buf[5]);
	tm->tm_year	= bcd2bin(buf[6]) + 80;

	return 0;
}

static int em3027_set_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char buf[8];

	buf[0] = EM3027_REG_WATCH_SEC;
	buf[1] = bin2bcd(tm->tm_sec);
	buf[2] = bin2bcd(tm->tm_min);
	buf[3] = bin2bcd(tm->tm_hour);
	buf[4] = bin2bcd(tm->tm_mday);
	buf[5] = bin2bcd(tm->tm_wday);
	buf[6] = bin2bcd(tm->tm_mon);
	buf[7] = bin2bcd(tm->tm_year) & 0x7f;

	return em3027_write_register(client, buf, 8);
}

static int em3027_read_alarm(struct device *dev, struct rtc_wkalrm *wa)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char buf[7];
	int err;

	err = em3027_read_register(client, EM3027_REG_ALARM_SEC, buf, 7);
	if (err)
		return err;

	wa->time.tm_sec	 = bcd2bin(0x7f & buf[0]);
	wa->time.tm_min	 = bcd2bin(0x7f & buf[1]);
	wa->time.tm_hour = bcd2bin(0x7f & buf[2]);
	wa->time.tm_mday = bcd2bin(0x7f & buf[3]);
	wa->time.tm_wday = bcd2bin(0x7f & buf[4]);
	wa->time.tm_mon	 = bcd2bin(0x7f & buf[5]);
	wa->time.tm_year = bcd2bin(0x7f & buf[6]) + 80;

	err = em3027_read_register(client, EM3027_REG_IRQ_CTRL, buf, 2);
	if (err)
		return -EIO;

	wa->enabled = !!(buf[0] & EM3027_BIT_IRQ_ALARM);
	wa->pending = (buf[1] & EM3027_BIT_IRQ_ALARM) && wa->enabled;

	return 0;
}

static int em3027_set_alarm(struct device *dev, struct rtc_wkalrm *wa)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char buf[8];
	int err;

	buf[0] = EM3027_REG_ALARM_SEC;
	buf[1] = (wa->time.tm_sec > 0)	? 0x80 | bin2bcd(wa->time.tm_sec)  : 0;
	buf[2] = (wa->time.tm_min > 0)	? 0x80 | bin2bcd(wa->time.tm_min)  : 0;
	buf[3] = (wa->time.tm_hour > 0)	? 0x80 | bin2bcd(wa->time.tm_hour) : 0;
	buf[4] = (wa->time.tm_mday > 0)	? 0x80 | bin2bcd(wa->time.tm_mday) : 0;
	buf[5] = (wa->time.tm_wday > 0)	? 0x80 | bin2bcd(wa->time.tm_wday) : 0;
	buf[6] = (wa->time.tm_mon > 0)	? 0x80 | bin2bcd(wa->time.tm_mon)  : 0;
	buf[7] = (wa->time.tm_year > 0)	?
				0x80 | (bin2bcd(wa->time.tm_year) & 0x7f) : 0;

	err = em3027_write_register(client, buf, 8);
	if (err)
		return -EIO;

	if (wa->enabled) {
		err = em3027_update_register(client, EM3027_REG_IRQ_CTRL,
					     EM3027_BIT_IRQ_ALARM,
					     EM3027_BIT_IRQ_ALARM);
		if (err)
			return err;
	}

	return 0;
}

static int em3027_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct i2c_client *client = to_i2c_client(dev);

	return em3027_update_register(client, EM3027_REG_IRQ_CTRL,
				      EM3027_BIT_IRQ_ALARM,
				      (enabled ? EM3027_BIT_IRQ_ALARM : 0));
}

static const struct rtc_class_ops em3027_rtc_ops = {
	.read_time = em3027_get_time,
	.set_time = em3027_set_time,
	.read_alarm = em3027_read_alarm,
	.set_alarm = em3027_set_alarm,
	.alarm_irq_enable = em3027_alarm_irq_enable,
};

static irqreturn_t em3027_irq_worker(int irq, void *dev_id)
{
	struct i2c_client *client = (struct i2c_client *)dev_id;
	struct rtc_device *rtc = i2c_get_clientdata(client);
	unsigned char irq_flags;
	int err;

	mutex_lock(&rtc->ops_lock);
	err = em3027_read_register(client, EM3027_REG_IRQ_FLAGS, &irq_flags, 1);
	if (err)
		goto done;

	if (irq_flags & EM3027_BIT_IRQ_ALARM) {
		/* notify top layers */
		rtc_update_irq(rtc, 1, RTC_IRQF | RTC_AF);
		irq_flags &= ~EM3027_BIT_IRQ_ALARM;
	}

	if (irq_flags) {
		dev_dbg(&client->dev,
			"ignored IRQ(s) mask: %02x \n", irq_flags);
		irq_flags = 0;
	}

	em3027_update_register(client, EM3027_REG_IRQ_FLAGS, EM3027_IRQ_MASK,
			       irq_flags);

done:
	mutex_unlock(&rtc->ops_lock);
	return IRQ_HANDLED;
}

static int em3027_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct rtc_device *rtc;
	struct em3027_platform_data *pdata = client->dev.platform_data;
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	device_init_wakeup(&client->dev, client->irq > 0);
	rtc = rtc_device_register(em3027_driver.driver.name, &client->dev,
				  &em3027_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	i2c_set_clientdata(client, rtc);

	if (pdata) {
		/* select trickle charger resistors */
		err = em3027_update_eeprom_register(client,
						EM3027_REG_EEPROM_CTRL,
						EM3027_TRICKLE_CHARGER_MASK,
						pdata->charger_resistor_sel);
		if (err)
			goto fail;
	}

	if (client->irq > 0) {
		/* configure rtc output as irq */
		err = em3027_update_register(client, EM3027_REG_ON_OFF_CTRL,
					     EM3027_BIT_ON_OFF_CLK_nINT, 0);
		if (err)
			goto fail;

		/* clear pending interrupts */
		err = em3027_update_register(client, EM3027_REG_IRQ_FLAGS,
					     EM3027_IRQ_MASK, 0);
		if (err)
			goto fail;

		err = request_threaded_irq(client->irq, NULL, em3027_irq_worker,
					   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					   em3027_driver.driver.name, client);
		if (err != 0) {
			dev_err(&client->dev, "%s: could not request irq %d \n",
				__func__, client->irq);
			goto fail;
		}
	}

	return 0;

fail:
	rtc_device_unregister(rtc);

	return err;
}

static int em3027_remove(struct i2c_client *client)
{
	struct rtc_device *rtc = i2c_get_clientdata(client);

	if (client->irq > 0) {
		em3027_update_register(client, EM3027_REG_IRQ_CTRL,
				       EM3027_BIT_IRQ_ALARM, 0);
		free_irq(client->irq, client);
	}

	rtc_device_unregister(rtc);

	return 0;
}

#ifdef CONFIG_PM
static int em3027_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int em3027_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	return 0;
}

#else
#define em3027_suspend	NULL
#define em3027_resume	NULL
#endif

static struct i2c_device_id em3027_id[] = {
	{ "em3027", 0 },
	{ }
};

static const struct dev_pm_ops em3027_pm_ops = {
	.suspend = em3027_suspend,
	.resume = em3027_resume,
};

static struct i2c_driver em3027_driver = {
	.driver = {
		.name = "rtc-em3027",
		.pm = &em3027_pm_ops,
	},
	.probe = em3027_probe,
	.remove = em3027_remove,
	.id_table = em3027_id,
};

static int __init em3027_init(void)
{
	return i2c_add_driver(&em3027_driver);
}

static void __exit em3027_exit(void)
{
	i2c_del_driver(&em3027_driver);
}

MODULE_AUTHOR("Mike Rapoport <mike@compulab.co.il>");
MODULE_DESCRIPTION("EM Microelectronic EM3027 RTC driver");
MODULE_LICENSE("GPL");

module_init(em3027_init);
module_exit(em3027_exit);
