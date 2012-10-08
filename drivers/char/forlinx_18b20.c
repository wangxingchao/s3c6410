//#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <linux/cdev.h>
#include <linux/device.h>

#include <linux/gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/regs-gpio.h>

#define DEVICE_NAME	"TEM"
#define tp_MAJOR  232


unsigned char sdata;
unsigned char xiaoshu1;
unsigned char xiaoshu2;
unsigned char xiaoshu;

void tmreset (void)
{      

	s3c_gpio_cfgpin(S3C64XX_GPE(0), S3C_GPIO_SFN(1));
	gpio_set_value(S3C64XX_GPE(0), 1);
	udelay(100);
	gpio_set_value(S3C64XX_GPE(0), 0);
	udelay(600);
	gpio_set_value(S3C64XX_GPE(0), 1);
	udelay(100);
	s3c_gpio_cfgpin(S3C64XX_GPE(0), S3C_GPIO_SFN(0));
}  

void tmwbyte (unsigned char dat)
{                       
	unsigned char j;
	s3c_gpio_cfgpin(S3C64XX_GPE(0), S3C_GPIO_SFN(1));
	for (j=1;j<=8;j++)      
	{ 
		gpio_set_value(S3C64XX_GPE(0), 0); 
		udelay(1); 
		if((dat&0x01)==1)
		{
			gpio_set_value(S3C64XX_GPE(0), 1);	  	  
		}	  
		else 
		{

		}
		udelay(60);
		gpio_set_value(S3C64XX_GPE(0), 1);
		udelay(15);
		dat = dat >> 1;
	}  
	gpio_set_value(S3C64XX_GPE(0), 1);
} 

unsigned char tmrbyte (void)
{   
	unsigned char i,u=0;      

	for (i=1;i<=8;i++)      
	{

		s3c_gpio_cfgpin(S3C64XX_GPE(0), S3C_GPIO_SFN(1));
		gpio_set_value(S3C64XX_GPE(0), 0); 
		udelay(1);
		u >>= 1; 
		gpio_set_value(S3C64XX_GPE(0), 1);
		s3c_gpio_cfgpin(S3C64XX_GPE(0), S3C_GPIO_SFN(0)); 
		if( gpio_get_value(S3C64XX_GPE(0)))    u=u|0x80;
		udelay(60);  
	}  
	return (u);   
}

void DS18B20PRO(void)         
{    
	unsigned char a,b; 
	tmreset();         
	udelay(420);
	tmwbyte(0xcc);        
	tmwbyte(0x44);      
	mdelay(750);
	tmreset ();    
	udelay(400);
	tmwbyte (0xcc);  
	tmwbyte (0xbe);  
	a = tmrbyte ();
	b= tmrbyte ();
	sdata = a/16+b*16;   
}	

static ssize_t  s3c6410_18b20_read(struct file *filp, char *buf, size_t len, loff_t *off)

{
	DS18B20PRO();
	*buf=sdata;
	return 1;
}

static struct file_operations s3c6410_18b20_fops = {
	.owner	=	THIS_MODULE,
	.read	=	s3c6410_18b20_read,
};

static struct cdev cdev_18b20;

static int __init s3c6410_18b20_init(void)
{
	int result;
	dev_t devno = MKDEV(tp_MAJOR,0);
	struct class *tem_class;

	result = register_chrdev_region(devno,1,DEVICE_NAME);

	if(result)
	{
		printk(KERN_NOTICE "Error %d register 18b20",result);
		return result;
	}

    
 	cdev_init(&cdev_18b20,&s3c6410_18b20_fops);

	result = cdev_add(&cdev_18b20,devno,1);
	if(result)
	{
		printk(KERN_NOTICE "Error %d adding 18b20",result);
		return result;
	}

	tem_class = class_create(THIS_MODULE, "tem_class");
	device_create(tem_class, NULL, MKDEV(tp_MAJOR, 0), "ds18b20","TEM%d", 0);


	return 0;	
}

static void __exit s3c6410_18b20_exit(void)
{
	cdev_del(&cdev_18b20);
	unregister_chrdev_region(MKDEV(tp_MAJOR,0),1);
}


module_init(s3c6410_18b20_init);
module_exit(s3c6410_18b20_exit);

MODULE_LICENSE("GPL");
