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
#include <linux/fs.h>
#include <linux/cdev.h>
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
#include <linux/sysfs.h>
#include <linux/device.h>

#include <linux/mtd/cfi.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <asm/uaccess.h>
#include <linux/poll.h>		/* for POLLIN, etc. */

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
static struct class *spifpga_class;
static int spi_test_aa55(void);
static int spi_u14_measure(void);
static int spi_measure_data(u8 addr);
int loop;
static struct cdev spifpga_cdev;  /* use 1 cdev for all pins */
struct timer_list fpga_timer;
struct spifpga {
	struct spi_device	*spi;
	struct mutex		lock;
	char			command[10];
};

struct spifpga *fpga_flash;
static ssize_t show_aa55(struct device *d,
		struct device_attribute *attr, char *buf)
{
	unsigned long ret_val;
	int i;
	for (i=0; i<loop; i++)
		ret_val = spi_test_aa55();
	return sprintf(buf, "0x%lX\n", ret_val);
}
static ssize_t show_temp(struct device *d,
		struct device_attribute *attr, char *buf)
{
	unsigned long ret_val;
	ret_val = spi_u14_measure();
	return sprintf(buf, "0x%lX\n", ret_val);
}
static ssize_t show_stress(struct device *d,
		struct device_attribute *attr, char *buf)
{
	unsigned long ret_val=0;
	printk(KERN_INFO"Show Pressure value\n");
	return sprintf(buf, "0x%lX\n", ret_val);
}

u8 fpga_address;
static ssize_t show_address(struct device *d,
		struct device_attribute *attr, char *buf)
{
	unsigned long ret_val=0;
	ret_val = spi_measure_data(fpga_address);
	printk(KERN_INFO "We are measuring %d addr value %d\n", fpga_address, ret_val);
	return sprintf(buf, "0x%lX\n", ret_val);
}
static ssize_t store_addr(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	unsigned long addr;
	sscanf(buf, "%lX", &addr);
	printk(KERN_INFO "Store %x As new Address\n", addr);
	fpga_address = addr&0xFF;
	return count;
}
static ssize_t show_loop(struct device *d,
		struct device_attribute *attr, char *buf)
{
	unsigned long ret_val=0;
	printk(KERN_INFO "Loop Value %d\n", loop);
	return sprintf(buf, "0x%lX\n", ret_val);
}

static ssize_t store_loop(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	sscanf(buf, "%lX", &loop);
	printk(KERN_INFO "Store %x As new Loop value\n", loop);
	return count;
}
static DEVICE_ATTR(fpga_test, S_IRUGO, show_aa55, NULL);
static DEVICE_ATTR(fpga_temp, S_IRUGO, show_temp, NULL);
static DEVICE_ATTR(fpga_stress, S_IRUGO, show_stress, NULL);
static DEVICE_ATTR(fpga_addr, S_IRUGO | S_IWUGO, show_address, store_addr);
static DEVICE_ATTR(fpga_loop, S_IRUGO | S_IWUGO, show_loop, store_loop);
static struct attribute *fpga_sysfs_entries[] = {
	&dev_attr_fpga_test.attr,
	&dev_attr_fpga_temp.attr,
	&dev_attr_fpga_stress.attr,
	&dev_attr_fpga_addr.attr,
	&dev_attr_fpga_loop.attr,
	NULL
};
static struct attribute_group fpga_attribute_group = {
	.name = NULL,		/* put in device directory */
	.attrs = fpga_sysfs_entries,
};

/* Read: 0 << 16*/
static int read_fpga(u16 addr, struct spi_device *spi)
{
	u16 code;
	u16 val;
	ssize_t retval;

	//code = (addr | (1<<16)) << 16; 
	code = (addr | (0<<16)); 
	//spi_write(spi, &code, 1);
	retval = spi_write_then_read(spi, (const u8 *)&code, sizeof(code), &val, sizeof(val));
	return val;
}

/* Write: 1 << 16*/
static int write_fpga(u16 addr, u16 val, struct spi_device *spi)
{
	u32 code;
	ssize_t retval;

	code = (addr | (1<<16)) << 16; 
	code |= val;
	spi_write(spi, &code, sizeof(code));
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

static int spi_u14_measure(void)
{
	u32 value;
	u16 addr;
	int retval;

	addr = 1;
	//retval = spi_write_then_read(spi_fpga, &code, 1, &val, 1);
	value = read_fpga(addr, spi_fpga);
	value = value & DATA_MASK; 
	printk(KERN_INFO "FPGA: read U14 measure data: 0x%x\n", value);
	return value;
}
static int spi_test_aa55(void)
{
	u32 value=0;
	u16 addr;
	int retval;

	addr = 0;
	//retval = spi_write_then_read(spi_fpga, &code, 1, &val, 1);
	value = read_fpga(addr, spi_fpga);
	value = value & DATA_MASK; 
	printk(KERN_INFO "FPGA: read aa55 value: 0x%x\n", value);
	return value;
}

static ssize_t
spifpga_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	return 0;
}

static int spifpga_open(struct inode *inode, struct file *file)
{
	return 0;
}
static long spifpga_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	return 0;
}
static unsigned int spifpga_poll(struct file *file, poll_table *wait)
{
	return 0;
}
static ssize_t spifpga_write(struct file *file, const char __user *buf,
			  size_t count, loff_t * ppos)
{
	return 0;
}

static int spifpga_release(struct inode *inode, struct file *file)
{
	return 0;
}
static const struct file_operations spifpga_fileops = {
	.owner   = THIS_MODULE,
	.write   = spifpga_write,
	.read    = spifpga_read,
	.open    = spifpga_open,
	.poll	 = spifpga_poll,
	.unlocked_ioctl	= spifpga_ioctl,
	.release = spifpga_release,
};

static void spi_fpga_timer_func(unsigned long data)
{
	printk(KERN_INFO "Add Timer functioner\n");
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
	dev_t devid;
	int major;

	spi_fpga = spi;
	printk(KERN_INFO "SPI: Probe SPI FPGA\n");
	fpga_flash = kzalloc(sizeof *fpga_flash, GFP_KERNEL);

	ret = alloc_chrdev_region(&devid, 0, 2, "spi-fpga");
	if (ret < 0)
		printk(KERN_ERR "%s: failed to allocate char dev region\n",
			__FILE__);
	major = MAJOR(devid);
	cdev_init(&spifpga_cdev, &spifpga_fileops);
	cdev_add(&spifpga_cdev, devid, 2);
#if 0
	/* Initialize SPI GPIO for FPGA */
	ret = gpio_request(S3C64XX_GPC(3), "fpga");
	if (ret) {
		printk(KERN_ERR "SPI: Setup CS-Pin0 Error\n");
	}
	s3c_gpio_setpull(S3C64XX_GPC(3), S3C_GPIO_PULL_NONE);	// Manual chip select pin as used in 6410_set_cs
	s3c_gpio_cfgpin(S3C64XX_GPC(3), S3C_GPIO_OUTPUT);		// Manual chip select pin as used in 6410_set_cs
#endif

#if 0
	for (i=0; i<100; i++)
		spi_test_aa55();
	spi_u14_measure();
#endif
	ret = sysfs_create_group(&spi->dev.kobj,
			&fpga_attribute_group);
	if (ret) {
		printk(KERN_INFO "FPGA: failed to create sysfs device attributes.\n");
	}

	/* Add Timer */
	setup_timer(&fpga_timer, spi_fpga_timer_func, spi);
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
