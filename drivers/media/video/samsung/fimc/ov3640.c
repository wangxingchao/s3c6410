/* linux/drivers/media/video/samsung/ov3640.c
 *
 * Omnivision OV3640 CMOS Image Sensor driver
 *
 * Figo Wang, Copyright (c) 2010
 * 	sagres_2004@163.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/init.h>
#include <asm/io.h>

#include "s3c_fimc.h"
#include "ov3640.h"

#define OV3640_I2C_ADDR		0x78

const static u16 ignore[] = { I2C_CLIENT_END };
const static u16 normal_addr[] = { (OV3640_I2C_ADDR >> 1), I2C_CLIENT_END };
const static u16 *forces[] = { NULL };
static struct i2c_driver ov3640_i2c_driver;

static struct s3c_fimc_camera ov3640_data = {
	.id 		= CONFIG_VIDEO_FIMC_CAM_CH,
	.type		= CAM_TYPE_ITU,
	.mode		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.clockrate	= 24000000,
	.width		= 640,
	.height		= 480,
	.offset		= {
		.h1 = 0,
		.h2 = 0,
		.v1 = 0,
		.v2 = 0,
	},

	.polarity	= {
		.pclk	= 0,
		.vsync	= 0,
		.href	= 0,
		.hsync	= 0,
	},

	.initialized	= 0,
};

static struct i2c_client_address_data addr_data = {
	.normal_i2c	= normal_addr,
	.probe		= ignore,
	.ignore		= ignore,
	.forces		= forces,
};

static void ov3640_start(struct i2c_client *client)
{
	int i;

	for (i = 0; i < sizeof(ov3640_setting_15fps_VGA_640_480) / sizeof(ov3640_setting_15fps_VGA_640_480[0]); i++) {
		s3c_fimc_i2c_write(client, ov3640_setting_15fps_VGA_640_480[i],sizeof(ov3640_setting_15fps_VGA_640_480[i]));
	}
}

static int ov3640_attach(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *c;

	c = kmalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	memset(c, 0, sizeof(struct i2c_client));	

	strcpy(c->name, "ov3640");
	c->addr = addr;
	c->adapter = adap;
	c->driver = &ov3640_i2c_driver;

	ov3640_data.client = c;

	printk("ov3640 attached successfully\n");

	return i2c_attach_client(c);
}

static int ov3640_attach_adapter(struct i2c_adapter *adap)
{
	int ret = 0;

	s3c_fimc_register_camera(&ov3640_data);

	ret = i2c_probe(adap, &addr_data, ov3640_attach);
	if (ret) {
		err("failed to attach ov3640 driver\n");
		ret = -ENODEV;
	}

	return ret;
}

static int ov3640_detach(struct i2c_client *client)
{
	i2c_detach_client(client);

	return 0;
}

static int ov3640_change_resolution(struct i2c_client *client, int res)
{
	switch (res) {
	case CAM_RES_DEFAULT:	/* fall through */
	case CAM_RES_MAX:	/* fall through */
		break;

	default:
		err("unexpect value\n");
	}

	return 0;
}

static int ov3640_command(struct i2c_client *client, u32 cmd, void *arg)
{
	switch (cmd) {
	case I2C_CAM_INIT:
		ov3640_start(client);
		printk("external camera initialized\n");
		break;

	case I2C_CAM_RESOLUTION:
		ov3640_change_resolution(client, (int) arg);
		break;

	default:
		err("unexpect command\n");
		break;
	}

	return 0;
}

static struct i2c_driver ov3640_i2c_driver = {
	.driver = {
		.name = "ov3640",
	},
	.id = I2C_DRIVERID_OV3640,
	.attach_adapter = ov3640_attach_adapter,
	.detach_client = ov3640_detach,
	.command = ov3640_command,
};

static __init int ov3640_init(void)
{
	printk("ov3640 camera initialized\n");  
	return i2c_add_driver(&ov3640_i2c_driver);
}

static __init void ov3640_exit(void)
{
	i2c_del_driver(&ov3640_i2c_driver);
}

module_init(ov3640_init)
module_exit(ov3640_exit)

MODULE_AUTHOR("Figo Wang <sagres_2004@163.com>");
MODULE_DESCRIPTION("Omnivision OV3640 I2C based CMOS Image Sensor driver");
MODULE_LICENSE("GPL");

