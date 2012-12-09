/*
 * MTD SPI driver for SPI FPGA serial flash chips
 *
 * Author: Wang Xingchao wxc200@gmail.com 
 *
 * Copyright (c) 2012.
 *
 * Part of code followed m25p80.c
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
#include <linux/miscdevice.h>
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
#define AA55		0x0
#define U14		0x1
#define ADDR_PRESSURE	0x2
#define ADDR_INK_TEMP	0x3
#define WR_EN		0x4
#define ADDR_BOARD_TEMP	0x5

#define CMD(l, h) (l&CMD_MASK | ((h&CMD_MASK)<<4))

/*Debug level: level 0 --> silent level 1 --> less log... level 5: blood log */
static int fpga_debug_level = 2;
module_param_named(level, fpga_debug_level, int, 0);
MODULE_PARM_DESC(level, "set fpga debug level");

#define fpga_db(format, args...) \
	if (fpga_debug_level > 2) \
		printk(format,  ##args)

static struct spi_device *spi_fpga;
static struct class *spifpga_class;
static int spi_test_aa55(void);
static int spi_u14_measure(void);
static int spi_measure_data(u8 addr);
static int write_fpga(u16 addr, u16 val, struct spi_device *spi);
static int read_fpga(u16 addr, struct spi_device *spi);
int loop = 1;
int write_value;
struct timer_list fpga_timer;
u8 fpga_address = 0x4;
char buffer[32] = {10, 20, 30};

struct spifpga {
	struct spi_device	*spi;
	struct mutex		lock;
	u8			command[10];
	u8			buffer[10];
};

struct fpga_data {
	u32 inktemp;
	u32 pressure;
	u32 temp;
	u32 u14;
	u32 timestamp;
};

struct spifpga *fpga_flash;

static ssize_t show_aa55(struct device *d,
		struct device_attribute *attr, char *buf)
{
	unsigned long ret_val;
	int i;
	for (i=0; i<loop; i++)
		ret_val = spi_test_aa55();
	return sprintf(buf, "0x%x\n", ret_val);
}
static ssize_t show_write_test(struct device *d,
		struct device_attribute *attr, char *buf)
{
	unsigned long ret_val;
	int i;
	int value = 0;
	u16 tmp[3] = {0x1234, 0x5678, 0xaa55};
	for (i=0; i<loop; i++) {
		write_fpga(fpga_address, tmp[i%3], spi_fpga);
		printk(KERN_INFO "SPI: Finish write test, read back test\n");
		msleep(100);
		value = read_fpga(fpga_address, spi_fpga);
		printk(KERN_INFO "FPGA: read Addr:0x%x value: 0x%x\n", fpga_address, value);
	}
#if 0
	printk(KERN_INFO "SPI: Finish write test, read back test\n");
	for (i=0; i<loop; i++) {
		value = read_fpga(fpga_address, spi_fpga);
		printk(KERN_INFO "FPGA: read Addr:%d value: 0x%x\n", fpga_address, value);
	}
#endif
	return sprintf(buf, "0x%x\n", value);
}
static ssize_t show_u14(struct device *d,
		struct device_attribute *attr, char *buf)
{
	unsigned long ret_val;
	ret_val = spi_u14_measure();
	return sprintf(buf, "0x%x\n", ret_val);
}
static ssize_t show_inktemp(struct device *d,
		struct device_attribute *attr, char *buf)
{
	unsigned long ret_val;
	int value;
	ret_val = spi_measure_data(ADDR_INK_TEMP);
	printk(KERN_INFO"Show ink temp value: %d\n", ret_val);
	value = ret_val;
	value = value >> 4;
	value = value & 0xFF;
	printk(KERN_INFO"Show ink tempture value(bit4~bit11): 0x%x\n", value);
	return sprintf(buf, "0x%x\n", ret_val);
}
static ssize_t show_temp(struct device *d,
		struct device_attribute *attr, char *buf)
{
	unsigned long ret_val;
	ret_val = spi_measure_data(ADDR_BOARD_TEMP);
	return sprintf(buf, "0x%x\n", ret_val);
}
static ssize_t show_stress(struct device *d,
		struct device_attribute *attr, char *buf)
{
	unsigned long ret_val=0;
	int value;
	ret_val = spi_measure_data(ADDR_PRESSURE);
	printk(KERN_INFO"Show Pressure value: %d\n", ret_val);
	value = ret_val;
	value = value >> 4;
	value = value & 0xFF;
	printk(KERN_INFO"Show Pressure value(bit4~bit11): 0x%x\n", value);
	return sprintf(buf, "0x%x\n", ret_val);
}

static ssize_t show_address(struct device *d,
		struct device_attribute *attr, char *buf)
{
	unsigned long ret_val=0;
	int i;
	for (i=0; i<loop; i++) {
		ret_val = spi_measure_data(fpga_address);
		printk(KERN_INFO "We are measuring %d addr value %d\n", fpga_address, ret_val);
	}
	return sprintf(buf, "0x%x\n", ret_val);
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
	return sprintf(buf, "0x%x\n", ret_val);
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
static ssize_t store_write_value(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	sscanf(buf, "%lX", &write_value);
	printk(KERN_INFO "Store %x As new write_value value\n", write_value);
	printk(KERN_INFO "Will Write 0x%x to address 0x%x\n", write_value, fpga_address);
	write_fpga(fpga_address, write_value, spi_fpga);
	return count;
}

static ssize_t show_write_value(struct device *d,
		struct device_attribute *attr, char *buf)
{
	unsigned long ret_val=0;
	printk(KERN_INFO "write_value= %d\n", write_value);
	return sprintf(buf, "0x%lX\n", write_value);
}

static DEVICE_ATTR(fpga_read_test, S_IRUGO, show_aa55, NULL);
static DEVICE_ATTR(fpga_write_test, S_IRUGO, show_write_test, NULL);
static DEVICE_ATTR(fpga_temp, S_IRUGO, show_temp, NULL);
static DEVICE_ATTR(fpga_inktemp, S_IRUGO, show_inktemp, NULL);
static DEVICE_ATTR(fpga_u14, S_IRUGO, show_u14, NULL);
static DEVICE_ATTR(fpga_stress, S_IRUGO, show_stress, NULL);
static DEVICE_ATTR(fpga_addr, S_IRUGO | S_IWUGO, show_address, store_addr);
static DEVICE_ATTR(fpga_loop, S_IRUGO | S_IWUGO, show_loop, store_loop);
static DEVICE_ATTR(fpga_write_value, S_IRUGO | S_IWUGO, show_write_value, store_write_value);
static struct attribute *fpga_sysfs_entries[] = {
	&dev_attr_fpga_read_test.attr,
	&dev_attr_fpga_write_test.attr,
	&dev_attr_fpga_temp.attr,
	&dev_attr_fpga_inktemp.attr,
	&dev_attr_fpga_u14.attr,
	&dev_attr_fpga_stress.attr,
	&dev_attr_fpga_addr.attr,
	&dev_attr_fpga_loop.attr,
	&dev_attr_fpga_write_value.attr,
	NULL
};
static struct attribute_group fpga_attribute_group = {
	.name = NULL,		/* put in device directory */
	.attrs = fpga_sysfs_entries,
};

/* Read: 0 << 16, First lower byte, then higher byte*/
static int read_fpga(u16 addr, struct spi_device *spi)
{
	u16 val=0;
	ssize_t retval;

	u8	cmd[2];
	u8 	buf[2];

	cmd[1] = (0<<7) | 0;   
	cmd[0] = addr & 0xFF;

	retval =  spi_write(spi, cmd, 2);
	if (retval < 0) {
		printk(KERN_INFO "SPI write error\n");
		goto out;
	}

	retval = spi_read(spi, buf, 2);
	if (retval < 0) {
		printk(KERN_INFO "SPI read error\n");
		goto out;
	}

	val = buf[0] | buf[1] << 8;
	return val;
out:
	return -1;
}

/* Write: 1 << 16*/
static int write_fpga(u16 addr, u16 val, struct spi_device *spi)
{
	ssize_t retval;
	u8* cmd = fpga_flash->command;

	cmd[1] = 1 << 7;   
	cmd[0] = addr & 0xFF;
	cmd[3] = (val >> 8) & 0xFF;
	cmd[2] = val & 0xFF;
	printk(KERN_INFO "SPI: write 0x%x to Addr 0x%x\n", val, addr);
	retval = spi_write(spi, cmd, 4);
	if (retval < 0) {
		printk(KERN_INFO "SPI write error\n");
	}
	return retval;
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
	u16 value=0;
	u16 addr;
	int retval;

	addr = 0;
	//retval = spi_write_then_read(spi_fpga, &code, 1, &val, 1);
	value = read_fpga(addr, spi_fpga);
	printk(KERN_INFO "FPGA: read aa55 value: 0x%x\n", value);
	return value;
}

static ssize_t
spifpga_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int ret;
	printk(KERN_INFO "SPI FPGA read operation\n");
	ret = copy_to_user(buf, buffer, count) ? -EFAULT : ret;
	// we only return one 32bit data now
	return ret;
}

static int spifpga_open(struct inode *inode, struct file *file)

{
	printk(KERN_INFO "SPI FPGA open operation, Qt Join!\n");
	return 0;
}
#define SPIFPGA_READ_INK_TEMP	0x1
#define SPIFPGA_READ_BOARD_TEMP	0x2
#define SPIFPGA_READ_PRESSURE	0x4
#define SPIFPGA_READ_TEST	0x0

#define SPIFPGA_ADDR		0x10
#define SPIFPGA_LOOP		0x11
#define SPIFPGA_GET_DATA	0x12
#define SPIFPGA_WRITE_DATA	0x13

/* update local buffer */
#define SPIFPGA_UPDATE_BUFFER	0x20

static long spifpga_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int i;
	int ret_val;
	int __user *argp = (int __user *)arg; //note this pointer
	int value;
	printk(KERN_INFO "ioctl, cmd=0x%02x, arg=0x%02lx\n",
		cmd, arg);
	switch(cmd) {
		case SPIFPGA_READ_TEST:
			for (i=0; i<loop; i++)
				ret_val = spi_test_aa55();
			break;
		case SPIFPGA_READ_INK_TEMP:
			value = spi_measure_data(ADDR_INK_TEMP);
			value = value >> 4;
			value = value & 0xFF;
			break;
		case SPIFPGA_READ_PRESSURE:
			value = spi_measure_data(ADDR_PRESSURE);
			value = value >> 4;
			value = value & 0xFF;
			break;
		case SPIFPGA_READ_BOARD_TEMP:
			value = spi_measure_data(ADDR_BOARD_TEMP);
			break;
		case SPIFPGA_ADDR:
			get_user(value, argp);
			fpga_address = value&0xff;
			break;
		case SPIFPGA_LOOP:
			get_user(value, argp);
			loop = value;
			break;
		case SPIFPGA_GET_DATA:/* set address before this ioctl */
			value = spi_measure_data(fpga_address);
			break;
		case SPIFPGA_WRITE_DATA:
			get_user(value, argp);
			printk(KERN_INFO "Write CMD: 0x%x to Addr: 0x%x\n", value, fpga_address);
			write_fpga(fpga_address, value, spi_fpga);
			break;
		case SPIFPGA_UPDATE_BUFFER:
		       	/* There are so many parameters need to update, ioctl will cause some delay...just update with such command and read an single buffer*/
			printk(KERN_INFO "Update local buffer\n");
			break;
		default:
			printk(KERN_INFO "Not Supported Command\n");
			break;
	}
	buffer[0] = ret_val;
	//modify buffer size
	ret_val = copy_to_user(argp, &value, sizeof(value));
	return ret_val;
}
static unsigned int spifpga_poll(struct file *file, poll_table *wait)
{
	printk(KERN_INFO "Donot support poll callback yet\n");
	return 0;
}
static ssize_t spifpga_write(struct file *file, const char __user *buf,
			  size_t count, loff_t * ppos)
{
	printk(KERN_INFO "Donot support write callback yet\n");
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
static struct miscdevice s3c_fpga_miscdev = {
	minor:		250,
	name:		"s3c-fpga",
	fops:		&spifpga_fileops
};
/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int __devinit fpga_probe(struct spi_device *spi)
{
	unsigned			i;
	int ret;
	dev_t devid;

	spi_fpga = spi;
	spi->bits_per_word = 16;
	printk(KERN_INFO "SPI: Probe SPI FPGA\n");
	fpga_flash = kzalloc(sizeof *fpga_flash, GFP_KERNEL);

	ret = misc_register(&s3c_fpga_miscdev);

	ret = sysfs_create_group(&spi->dev.kobj,
			&fpga_attribute_group);
	if (ret) {
		printk(KERN_INFO "FPGA: failed to create sysfs device attributes.\n");
	}

	/* Add Timer to poll reading data*/
	setup_timer(&fpga_timer, spi_fpga_timer_func, spi);
	return 0;
}

/* unregister and free operations here*/
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
MODULE_AUTHOR("Wang Xingchao");
MODULE_DESCRIPTION("SPI FPGA Driver");
