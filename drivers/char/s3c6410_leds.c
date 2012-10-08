#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <asm/irq.h>
//#include <mach/regs-gpio.h>
#include <mach/hardware.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/unistd.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>

#include <plat/gpio-cfg.h>
#include <mach/gpio-bank-e.h>
#include <mach/gpio-bank-m.h>

#define DEVICE_NAME "leds"

static long s3c6410_leds_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch(cmd) {
		unsigned tmp;
	case 0:
	case 1:
                 if (arg > 4) 
                 {
		 	return -EINVAL;
		 }
		tmp = readl(S3C64XX_GPMDAT);
            
		if(cmd==0) //close light
                  { 
			tmp &= (~(1<<arg));
                  }
		else  //open light
                  { 
			tmp |= (1<<arg);
                  }

                writel(tmp,S3C64XX_GPMDAT);

		printk (DEVICE_NAME": %d %d\n", arg, cmd);
		return 0;
	default:
		return -EINVAL;
	}
}

static struct file_operations dev_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl	= s3c6410_leds_ioctl,
};

static struct miscdevice misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &dev_fops,
};

static int __init dev_init(void)
{
	int ret;
        
       unsigned tmp;

       //gpm0-3 pull up
	tmp = readl(S3C64XX_GPMPUD);
	tmp &= (~0xFF);
	tmp |= 0xaa;
	writel(tmp,S3C64XX_GPMPUD);

	//gpm0-3 output mode
	tmp =readl(S3C64XX_GPMCON);
	tmp &= (~0xFFFF);
	tmp |= 0x1111;
	writel(tmp,S3C64XX_GPMCON);
	
	//gpm0-3 output 0
	tmp = __raw_readl(S3C64XX_GPMDAT);
	tmp |= 0x10;
	writel(tmp,S3C64XX_GPMDAT);  

	ret = misc_register(&misc);
	printk ("\n@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	printk (DEVICE_NAME"\tinitialized\n");
	printk ("\n@@@@@@@@@@@@@@@@@@@@@@@@@@\n");

	return ret;
}

static void __exit dev_exit(void)
{
	misc_deregister(&misc);
}

module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("FORLINX Inc.");
