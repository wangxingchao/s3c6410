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
	u16 val;
	ssize_t retval;

	code = (addr | (1<<16)) << 16; 
	code |= val;
	spi_write(spi, &code, 1);
	return 0;
}

/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int __devinit fpga_probe(struct spi_device *spi)
{
	const struct spi_device_id	*id = spi_get_device_id(spi);
	struct flash_platform_data	*data;
	struct m25p			*flash;
	struct flash_info		*info;
	unsigned			i;
	struct mtd_partition		*parts = NULL;
	int				nr_parts = 0;

	printk(KERN_INFO "SPI: Probe SPI FPGA\n");

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
