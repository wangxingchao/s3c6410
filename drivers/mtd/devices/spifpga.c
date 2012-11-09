/*
 * MTD SPI driver for SPI FPGA (and similar) serial flash chips
 *
 * Author: Wang Xingchao wxc200@gmail.com 
 *
 * Copyright (c) 2012, Intel.
 *
 * Some parts are based on lart.c by Abraham Van Der Merwe
 *
 * Cleaned up and generalized based on mtd_dataflash.c
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/math64.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mod_devicetable.h>

#include <linux/mtd/cfi.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

/* FPGA CMD */
#define FPGA_R	0x0
#define FPGA_W	0x1
#define ADDR_MASK	0x7fff
#define DATA_MASK	0xffff
#define CMD_MASK		0xf

/* A4-A7:0 A0-A3 */
#define AA55	0x0
#define U14	0x1
#define U89	0x2
#define U90	0x3
#define WR_EN	0x4

#define CMD(l, h) (l&CMD_MASK | ((h&CMD_MASK)<<4))

static struct spi_device *spi_fpga;
static int read_fpga(u16 addr, struct spi_dev *spi)
{
	u32 code;
	u16 val;
	ssize_t retval;

	code = (addr | (1<<16)) << 16; 
	//spi_write(spi, &code, 1);
	retval = spi_write_then_read(spi, &code, 1, &val, 1);
	return val;
}

static int write_fpga(u16 addr, u16 val, struct spi_dev *spi)
{
	u32 code;
	ssize_t retval;

	code = (addr | (1<<16)) << 16; 
	code |= val;
	spi_write(spi, &code, 1);
	return 0;
}

static int spi_measure_data(u8 addr)
{
	u32 value;
	u16 address = 0;
	int retval;

	address |= addr;
	value = read_fpga(address, spi_fpga);
	value = value & DATA_MASK; 
	return value;
}

static void spi_u14_measure(void)
{
	u32 value;
	u16 addr;
	int retval;

	addr = 1;
	//retval = spi_write_then_read(spi_fpga, &code, 1, &val, 1);
	value = read_fpga(addr, spi_fpga);
	value = value & DATA_MASK; 
	printk(KERN_INFO "FPGA: read U14 measure data: 0x%x\n", value);
}
static void spi_test_aa55(void)
{
	u32 value=0;
	u16 addr;
	int retval;

	addr = 0;
	//retval = spi_write_then_read(spi_fpga, &code, 1, &val, 1);
	value = read_fpga(addr, spi_fpga);
	value = value & DATA_MASK; 
	printk(KERN_INFO "FPGA: read aa55 value: 0x%x\n", value);
}

/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int __devinit fpga_probe(struct spi_device *spi)
{
	struct flash_platform_data	*data;
	struct flash_info		*info;
	unsigned			i;
	int ret;

	spi_fpga = spi;
	printk(KERN_INFO "SPI: Probe SPI FPGA\n");

#if 0
	/* Initialize SPI GPIO for FPGA */
	ret = gpio_request(S3C64XX_GPC(3), "fpga");
	if (ret) {
		printk(KERN_ERR "SPI: Setup CS-Pin0 Error\n");
	}
	s3c_gpio_setpull(S3C64XX_GPC(3), S3C_GPIO_PULL_NONE);	// Manual chip select pin as used in 6410_set_cs
	s3c_gpio_cfgpin(S3C64XX_GPC(3), S3C_GPIO_OUTPUT);		// Manual chip select pin as used in 6410_set_cs
#endif

	for (i=0; i<100; i++)
		spi_test_aa55();
	spi_u14_measure();
	return 0;
}


static int __devexit fpga_remove(struct spi_device *spi)
{
	return 0;
}


static struct spi_driver fpga_driver = {
	.driver = {
		.name	= "spi-fpga",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= fpga_probe,
	.remove	= __devexit_p(fpga_remove),

	/* REVISIT: many of these chips have deep power-down modes, which
	 * should clearly be entered on suspend() to minimize power use.
	 * And also when they're otherwise idle...
	 */
};


static int __init fpga_init(void)
{
	printk(KERN_INFO "SPI: SPI FPGA Driver Register\n");
	return spi_register_driver(&fpga_driver);
}


static void __exit fpga_exit(void)
{
	spi_unregister_driver(&fpga_driver);
}


module_init(fpga_init);
module_exit(fpga_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mike Lavender");
MODULE_DESCRIPTION("MTD SPI driver for ST M25Pxx flash chips");
