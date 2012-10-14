/*
 * lis3l02dq.c	support STMicroelectronics LISD02DQ
 *		3d 2g Linear Accelerometers via SPI
 *
 * Copyright (c) 2007 Jonathan Cameron <jic23@cam.ac.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Settings:
 * 16 bit left justified mode used.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

static int __devinit spi_eeprom_probe(struct spi_device *spi)
{
	unsigned char buf[0x20];
	u8 cmd[2];
	int ret;
	int i;

	printk(KERN_INFO "SPI: EEprom register\n");
	cmd[0] = 0x03;
	cmd[1] = 0x0; 
	ret = spi_write_then_read(spi, cmd, sizeof(cmd), buf, 0x20);
	if (ret < 0)
		printk(KERN_INFO "SPI: Spi write/read error\n");

	for (i=0; i<0x20; i++)
		printk(KERN_INFO "buf[%d] = 0x%x\n", i, buf[i]);

}

/* fixme, confirm ordering in this function */
static int spi_eeprom_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver spi_eeprom_driver = {
	.driver = {
		.name = "spi_eeprom",
		.owner = THIS_MODULE,
	},
	.probe = spi_eeprom_probe,
	.remove = __devexit_p(spi_eeprom_remove),
};

static __init int spi_eeprom_init(void)
{
	return spi_register_driver(&spi_eeprom_driver);
}
module_init(spi_eeprom_init);

static __exit void spi_eeprom_exit(void)
{
	spi_unregister_driver(&spi_eeprom_driver);
}
module_exit(spi_eeprom_exit);

MODULE_AUTHOR("Jonathan Cameron <jic23@cam.ac.uk>");
MODULE_DESCRIPTION("ST LIS3L02DQ Accelerometer SPI driver");
MODULE_LICENSE("GPL v2");
