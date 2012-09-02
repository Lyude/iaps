/*
 *  IdeaPad's version of HDAPS driver
 *
 *  Copyright (C) 2005 Robert Love <rml@novell.com>
 *  Copyright (C) 2005 Jesper Juhl <jesper.juhl@gmail.com>
 *  Copyright (C) 2010 Javier S. Pedro <maemo@javispedro.com>
 *  Copyright (C) 2012 Stephen Chandler Paul <thatslyude@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/input.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input-polldev.h>

#include <asm/io.h>

#define IAPS_LOW_PORT 0x702
#define IAPS_NR_PORTS 2

#define IAPS_ADDR_PORT 0x702
#define IAPS_DATA_PORT 0x703
#define IAPS_CTRL_PORT 0x6C
#define IAPS_ERR_PORT  0x68

#define IAPS_REG_CMD   0x10
#define IAPS_REG_POS_X 0x12
#define IAPS_REG_POS_Y 0x14

#define IAPS_CMD_ENABLE  0x11
#define IAPS_CMD_DISABLE 0x10

#define IAPS_POLL_INTERVAL	50	/* poll for input every 1/20s (50 ms)*/
#define IAPS_RANGE_MIN 400
#define IAPS_RANGE_MAX 740
#define IAPS_INPUT_FUZZ	6
#define IAPS_INPUT_FLAT	6

static struct input_polled_dev *idev;
static struct platform_device *pdev;

static inline u8 iaps_readb(u8 idx)
{
	outb(idx, IAPS_ADDR_PORT);
	return inb(IAPS_DATA_PORT);
}

static inline void iaps_writeb(u8 value, u8 idx)
{
	outb(idx, IAPS_ADDR_PORT);
	outb(value, IAPS_DATA_PORT);
}

/** Read a signed 16-byte value from device */
static short iaps_readw(u8 idx)
{
	union {
		char c[2];
		s16 s;
	} v;
	v.c[0] = iaps_readb(idx);
	v.c[1] = iaps_readb(idx+1);
	return v.s;
}

/** Send a "command" to the device */
static int iaps_command(u8 cmd)
{
	int timeout = 1000;

	iaps_writeb(cmd, IAPS_REG_CMD);

	/* Controller seems to be in error state, recover */
	if (inb(IAPS_CTRL_PORT) & 1) {
		inb(IAPS_ERR_PORT);
	}

	/* Wait until ready to accept commands */
	while ((inb(IAPS_CTRL_PORT) & 2) && (timeout > 0)) {
		udelay(50);
		timeout--;
	}

	/* Send command */
	outb(cmd, IAPS_CTRL_PORT);

	return timeout > 0 ? 0 : -ENXIO;
}

static int iaps_wait(void)
{
	int timeout = 1000;

	/* Wait until command is processed */
	while (!(inb(IAPS_CTRL_PORT) & 1) && timeout > 0) {
		udelay(50);
		timeout--;
	}

	return timeout > 0 ? 0 : -ENXIO;
}

static void iaps_poll(struct input_polled_dev *dev)
{
	struct input_dev *input_dev = dev->input;

	/* This causes a refresh */
	iaps_command(IAPS_CMD_ENABLE);
	iaps_wait();

	input_report_abs(input_dev, ABS_X, iaps_readw(IAPS_REG_POS_X));
	input_report_abs(input_dev, ABS_Y, iaps_readw(IAPS_REG_POS_Y));

	input_sync(input_dev);
}

static ssize_t hdaps_position_show(struct device *dev,
				   struct device_attribute *attr, char *buf) {
	return sprintf(buf, "(%d,%d)\n", iaps_readw(IAPS_REG_POS_X), iaps_readw(IAPS_REG_POS_Y));
}

static DEVICE_ATTR(position, 0444, hdaps_position_show, NULL);

static struct attribute *hdaps_attributes[] = {
	&dev_attr_position.attr,
	NULL,
};

static struct attribute_group hdaps_attribute_group = {
	.attrs = hdaps_attributes,
};

static int __init iaps_init(void) {
	int ret;

	if (!request_region(IAPS_LOW_PORT, IAPS_NR_PORTS, "iaps")) {
		ret = -ENXIO;
		goto out;
	}

	idev = input_allocate_polled_device();
	if (!idev) {
		ret = -ENOMEM;
		goto out_region;
	}

	idev->poll = iaps_poll;
	idev->poll_interval = IAPS_POLL_INTERVAL;

	/* Initialize the input class */
	idev->input->name = "IdeaPad HDAPS accelerometer data";
	idev->input->phys = "hdaps/input1";
	idev->input->id.bustype = BUS_HOST;
	idev->input->evbit[0] = BIT_MASK(EV_ABS);
	input_set_abs_params(idev->input, ABS_X,
			IAPS_RANGE_MIN, IAPS_RANGE_MAX, IAPS_INPUT_FUZZ, IAPS_INPUT_FLAT);
	input_set_abs_params(idev->input, ABS_Y,
			IAPS_RANGE_MIN, IAPS_RANGE_MAX, IAPS_INPUT_FUZZ, IAPS_INPUT_FLAT);

	ret = input_register_polled_device(idev);
	if (ret)
		goto out_idev;

	/* Register as platform device */
	pdev = platform_device_register_simple("hdaps", -1, NULL, 0);
	if (IS_ERR(pdev))
		goto out;

	ret = sysfs_create_group(&pdev->dev.kobj, &hdaps_attribute_group);
	if (ret)
		goto out;

	printk(KERN_INFO "iaps: driver initialized\n");
	return 0;

out_idev:
	input_free_polled_device(idev);
out_region:
	release_region(IAPS_LOW_PORT, IAPS_NR_PORTS);
out:
	printk(KERN_WARNING "iaps: driver init failed (ret=%d)!\n", ret);
	platform_device_unregister(pdev);
	return ret;
}

static void __exit iaps_exit(void) {
	iaps_command(IAPS_CMD_DISABLE);
	iaps_wait();

	input_unregister_polled_device(idev);
	input_free_polled_device(idev);
	release_region(IAPS_LOW_PORT, IAPS_NR_PORTS);
}

module_init(iaps_init);
module_exit(iaps_exit);

MODULE_LICENSE("GPL");

MODULE_AUTHOR("Javier S. Pedro");
MODULE_DESCRIPTION("Lenovo IdeaPad Active Protection System");

