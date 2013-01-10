/*
  touch.c : driver touchscreen
*/

#include <linux/module.h> // needed for modules
#include <linux/kernel.h> // needed for KERN_ALERT
#include <linux/init.h>   // needed for macros
#include <linux/fs.h>     // needed for driver management
#include <asm/uaccess.h>  // needed for copy_to/from_user
#include <asm/system.h>
#include <linux/types.h>
#include <linux/time.h>	  // needed for time services
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/slab.h>   // needed for kmalloc and kfree
#include "touch.h"

MODULE_AUTHOR("T Fourcroy - G Franciulla");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("driver touchsreen; kernel version : 2.6");

static int major = 42;   // allocation dynamique par défaut
static int minor = 0;

struct devInfo
{
	double x;
	double y;
	double pressure;
};

struct devInfo *touchscreen;

static ssize_t open(struct inode *inode, struct file *filp)
{
	struct devInfo *dev;
	
	dev = container_of (inode->i_cdev, struct devInfo, cdev);
	
	filp-> private_data = dev;
	return 0;
}

static ssize_t read(struct file *filp, char * buf, size_t count, loff_t *ppos)
{
  int result;
  struct devInfo *dev = filp->private_data;
  if (copy_to_user(buf, &dev->value, sizeof(int))) {
    result = -EFAULT;
    goto out;
  }
  result = sizeof(int);
  printk(KERN_ALERT "Read operation OK. X value: %d Y value: %d Pressure: %d\n", dev->x, dev->y, dev->pressure);
  
out:
  return result;
}

static ssize_t release(struct inode *inode, struct file* filp)
{
	printk(KERN_ALERT "touchscreen_close()\n");
	return 0;
}


static struct file_operations fops =
{
	.open =		open,
	.read = 	read,
	.release = 	release
};

static void setup_cdev(struct devInfo *dev, int index) 
{
	int err, devno=MKDEV(major, minor+index);
	
	cdev_init(&dev->cdev, &fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &fops;
	err = cdev_add(&dev->cdev, devno, 1);
	
	if (err) printk(KERN_ALERT "error %d adding my_devices%d\n", err, index);
}

static void desinstall(void)
{
	dev_t devno = MKDEV(major, minor);
	printk(KERN_ALERT "see you\n");
	
	kfree(device);
	unregister_chrdev_region(devno, MAXDEVICES);
	printk(KERN_ALERT "desinstallation OK\n");
}
static int __init install(void)
{
	int i, result;
	dev_t dev=0;

	printk(KERN_ALERT "Initialisation driver touchscreen\n");
	
	printk(KERN_ALERT "Install driver : dynamic allocation of major number\n");
	result = register_chrdev_region(&dev, MAXDEVICES, DRIVERNAME);

	if (result<0) {
	  printk(KERN_ALERT "install driver : can't get major number\n");
	  return result;
	}
	
	device = kmalloc(MAXDEVICES*sizeof(struct devInfo), GFP_KERNEL);
	if (!device) {
	    result = -ENOMEM;
	    goto fail;
	}
	memset(device, 0, MAXDEVICES*sizeof(struct devInfo));
	
	for (i=0; i < MAXDEVICES; i++) {
	  device[i].value = -1;
	  setup_cdev(&device[i], i);
	}

	initialisation();
	printk(KERN_ALERT "installation OK, MAJOR NUMBER: %i\n",major);
	return 0;
	
fail:
	desinstall();
	return result;
}

module_init(install);
module_exit(desinstall);


