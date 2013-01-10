/*
 *	Linux I2C driver for NXP LPC22XX
 *
 *	Copyright (C) 2007 Siemens Building Technologies
 *	                   mailto:philippe.goetz@siemens.com
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 * CONFIG_I2C_DEBUG_BUS => lpc22xx_i2c functions produce debugging info
 * CONFIG_I2C_DEBUG_ALGO => lpc22xx_pca functions produce debugging info
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include <asm/arch/board.h>

#include <linux/i2c.h>

#define DRIVER "lpc22xx-i2c"

// CONFIG_I2C_DEBUG_BUS => lpc22xx_i2c functions produce debugging info
// CONFIG_I2C_DEBUG_ALGO => lpc22xx_pca functions produce debugging info

struct lpc22xx_i2c_dev_data {
	int	own;	// own address
	int	clock;  // in Hz
};

/* This is reserved for the future
static unsigned long base = 0;
static int irq = -1;
*/

struct lpc22xx_i2c_regs {
	volatile unsigned long REG_I2C_I2CONSET;
	volatile unsigned long REG_I2C_I2STAT;
	volatile unsigned long REG_I2C_I2DAT;
	volatile unsigned long REG_I2C_I2ADR;
	volatile unsigned long REG_I2C_I2SCLH;
	volatile unsigned long REG_I2C_I2SCLL;
	volatile unsigned long REG_I2C_I2CONCLR;
};
#define KREG_I2C_I2CON_I2EN	0x40
#define KREG_I2C_I2CON_STA	0x20
#define KREG_I2C_I2CON_STO	0x10
#define KREG_I2C_I2CON_SI	0x08
#define KREG_I2C_I2CON_AA	0x04


struct lpc22xx_i2c_data {
	struct i2c_adapter adapter;
	struct lpc22xx_i2c_regs * base;
	int irq;
	int own;
	int clock;
	wait_queue_head_t wait;
};

static inline int lpc22xx_pca_status(struct lpc22xx_i2c_data *drv_data) {
	return drv_data->base->REG_I2C_I2STAT;
}

static inline int lpc22xx_pca_get_con(struct lpc22xx_i2c_data *drv_data) {
	return drv_data->base->REG_I2C_I2CONSET;
}

static inline void lpc22xx_pca_set_con(struct lpc22xx_i2c_data *drv_data, int set, int clr) {
	drv_data->base->REG_I2C_I2CONSET = set;
	drv_data->base->REG_I2C_I2CONCLR = clr;
}

static inline int lpc22xx_pca_get_dat(struct lpc22xx_i2c_data *drv_data) {
	return drv_data->base->REG_I2C_I2DAT;
}

static inline void lpc22xx_pca_set_dat(struct lpc22xx_i2c_data *drv_data, int data) {
	drv_data->base->REG_I2C_I2DAT = data;
}

static int lpc22xx_pca_init(struct lpc22xx_i2c_data *drv_data)
{
	int divider = LPC22xx_Fpclk / drv_data->clock;

	/* The divider shall be splited between the High and Low Duty Cycle Registers */
	drv_data->base->REG_I2C_I2SCLL = divider / 2;
	drv_data->base->REG_I2C_I2SCLH = divider / 2;

	/* This has to be programmed for the Slave mode */ 
	drv_data->base->REG_I2C_I2ADR = drv_data->own << 1;

	/* Master mode ONLY */
	lpc22xx_pca_set_con(drv_data, KREG_I2C_I2CON_I2EN, KREG_I2C_I2CON_STA|KREG_I2C_I2CON_STO|KREG_I2C_I2CON_SI|KREG_I2C_I2CON_AA);

	return 0;
}

static int lpc22xx_pca_wait(struct lpc22xx_i2c_data *drv_data)
{
	int ret = 0;

	if (drv_data->irq > -1) {
		ret = wait_event_interruptible(drv_data->wait,
		                               lpc22xx_pca_get_con(drv_data) & KREG_I2C_I2CON_SI);
	} else {
		while ((lpc22xx_pca_get_con(drv_data) & KREG_I2C_I2CON_SI) == 0) 
			udelay(100);
	}
	return ret;
}

/*
 * Generate a start condition on the i2c bus.
 *
 * returns after the start condition has occurred
 */
static void lpc22xx_pca_start(struct lpc22xx_i2c_data *drv_data)
{
#ifdef CONFIG_I2C_DEBUG_ALGO
	printk("=== START\n");
#endif
	lpc22xx_pca_set_con(drv_data, KREG_I2C_I2CON_STA, KREG_I2C_I2CON_STO|KREG_I2C_I2CON_SI);
	lpc22xx_pca_wait(drv_data);
}

/*
 * Generate a repeated start condition on the i2c bus
 *
 * return after the repeated start condition has occurred
 */
static void lpc22xx_pca_repeated_start(struct lpc22xx_i2c_data *drv_data)
{
#ifdef CONFIG_I2C_DEBUG_ALGO
	printk("=== REPEATED START\n");
#endif
	lpc22xx_pca_set_con(drv_data, KREG_I2C_I2CON_STA, KREG_I2C_I2CON_STO|KREG_I2C_I2CON_SI);
	lpc22xx_pca_wait(drv_data);
}

/*
 * Generate a stop condition on the i2c bus
 *
 * returns after the stop condition has been generated
 *
 * STOPs do not generate an interrupt or set the SI flag, since the
 * part returns the idle state (0xf8). Hence we don't need to
 * pca_wait here.
 */
static void lpc22xx_pca_stop(struct lpc22xx_i2c_data *drv_data)
{
#ifdef CONFIG_I2C_DEBUG_ALGO
	printk("=== STOP\n");
#endif
	lpc22xx_pca_set_con(drv_data, KREG_I2C_I2CON_STO, KREG_I2C_I2CON_STA|KREG_I2C_I2CON_SI);
}

/*
 * Send the slave address and R/W bit
 *
 * returns after the address has been sent
 */
static void lpc22xx_pca_address(struct lpc22xx_i2c_data *drv_data, 
			struct i2c_msg *msg)
{
	int addr = ( (0x7f & msg->addr) << 1 );

	if (msg->flags & I2C_M_RD )
		addr |= 1;

#ifdef CONFIG_I2C_DEBUG_ALGO
	printk("=== SLAVE ADDRESS %#04x+%c=%#04x\n", 
	     msg->addr, msg->flags & I2C_M_RD ? 'R' : 'W', addr);
#endif
	lpc22xx_pca_set_dat(drv_data, addr);

	lpc22xx_pca_set_con(drv_data, KREG_I2C_I2CON_AA, KREG_I2C_I2CON_STO|KREG_I2C_I2CON_STA|KREG_I2C_I2CON_SI);

	lpc22xx_pca_wait(drv_data);
}

/*
 * Transmit a byte.
 *
 * Returns after the byte has been transmitted
 */
static void lpc22xx_pca_tx_byte(struct lpc22xx_i2c_data *drv_data, 
			__u8 b)
{
#ifdef CONFIG_I2C_DEBUG_ALGO
	printk("=== WRITE %#04x\n", b);
#endif
	lpc22xx_pca_set_dat(drv_data, b);

	lpc22xx_pca_set_con(drv_data, KREG_I2C_I2CON_AA, KREG_I2C_I2CON_STO|KREG_I2C_I2CON_STA|KREG_I2C_I2CON_SI);

	lpc22xx_pca_wait(drv_data);
}

/*
 * Receive a byte
 *
 * returns immediately.
 */
static void lpc22xx_pca_rx_byte(struct lpc22xx_i2c_data *drv_data, 
			__u8 *b, int ack)
{
	*b = lpc22xx_pca_get_dat(drv_data);
#ifdef CONFIG_I2C_DEBUG_ALGO
	printk("=== READ %#04x %s\n", *b, ack ? "ACK" : "NACK");
#endif
}

/* 
 * Setup ACK or NACK for next received byte and wait for it to arrive.
 *
 * Returns after next byte has arrived.
 */
static void lpc22xx_pca_rx_ack(struct lpc22xx_i2c_data *drv_data, 
		       int ack)
{
	if ( ack ) {
		lpc22xx_pca_set_con(drv_data, KREG_I2C_I2CON_AA, KREG_I2C_I2CON_STO|KREG_I2C_I2CON_STA|KREG_I2C_I2CON_SI);
	}
	else {
		lpc22xx_pca_set_con(drv_data, 0, KREG_I2C_I2CON_STO|KREG_I2C_I2CON_STA|KREG_I2C_I2CON_SI|KREG_I2C_I2CON_AA);
	}
	lpc22xx_pca_wait(drv_data);
}

/* 
 * Reset the i2c bus / SIO 
 */
static void lpc22xx_pca_reset(struct lpc22xx_i2c_data *drv_data)
{
	printk(KERN_ERR DRIVER ": Haven't figured out how to do a reset yet\n");
	lpc22xx_pca_set_con(drv_data, 0, KREG_I2C_I2CON_I2EN);
	lpc22xx_pca_init(drv_data);
}

static int lpc22xx_pca_xfer(struct i2c_adapter *i2c_adap,
                    struct i2c_msg *msgs,
                    int num)
{
	struct lpc22xx_i2c_data *drv_data = i2c_adap->algo_data;
        struct i2c_msg *msg = NULL;
        int curmsg;
	int numbytes = 0;
	int state;
	int ret;
	int timeout = 100;

	while ((state = lpc22xx_pca_status(drv_data)) != 0xf8 && timeout--) {
		msleep(10);
	}
	if (state != 0xf8) {
		dev_dbg(&i2c_adap->dev, "bus is not idle. status is %#04x\n", state);
		return -EIO;
	}

#ifdef CONFIG_I2C_DEBUG_ALGO
	printk("{{{ XFER %d messages\n", num);
	for (curmsg = 0; curmsg < num; curmsg++) {
		int addr, i;
		msg = &msgs[curmsg];
		
		addr = (0x7f & msg->addr) ;
	
		if (msg->flags & I2C_M_RD )
			printk(KERN_INFO "    [%02d] RD %d bytes from %#02x [%#02x, ...]\n", 
			       curmsg, msg->len, addr, (addr<<1) | 1);
		else {
			printk(KERN_INFO "    [%02d] WR %d bytes to %#02x [%#02x%s", 
			       curmsg, msg->len, addr, addr<<1,
			       msg->len == 0 ? "" : ", ");
			for(i=0; i < msg->len; i++)
				printk("%#04x%s", msg->buf[i], i == msg->len - 1 ? "" : ", ");
			printk("]\n");
		}
	}
#endif

	curmsg = 0;
	ret = -EREMOTEIO;
	while (curmsg < num) {
		state = lpc22xx_pca_status(drv_data);

#ifdef CONFIG_I2C_DEBUG_ALGO
		printk("STATE is 0x%02x\n", state);
#endif
		msg = &msgs[curmsg];

		switch (state) {
		case 0xf8: /* On reset or stop the bus is idle */
			lpc22xx_pca_start(drv_data);
			break;

		case 0x08: /* A START condition has been transmitted */
		case 0x10: /* A repeated start condition has been transmitted */
			lpc22xx_pca_address(drv_data, msg);
			break;
			
		case 0x18: /* SLA+W has been transmitted; ACK has been received */
		case 0x28: /* Data byte in I2CDAT has been transmitted; ACK has been received */
			if (numbytes < msg->len) {
				lpc22xx_pca_tx_byte(drv_data, msg->buf[numbytes]);
				numbytes++;
				break;
			}
			curmsg++; numbytes = 0;
			if (curmsg == num)
				lpc22xx_pca_stop(drv_data);
			else
				lpc22xx_pca_repeated_start(drv_data);
			break;

		case 0x20: /* SLA+W has been transmitted; NOT ACK has been received */
#ifdef CONFIG_I2C_DEBUG_ALGO
			printk("NOT ACK received after SLA+W\n");
#endif
			lpc22xx_pca_stop(drv_data);
			goto out;

		case 0x40: /* SLA+R has been transmitted; ACK has been received */
			lpc22xx_pca_rx_ack(drv_data, msg->len > 1);
			break;

		case 0x50: /* Data bytes has been received; ACK has been returned */
			if (numbytes < msg->len) {
				lpc22xx_pca_rx_byte(drv_data, &msg->buf[numbytes], 1);
				numbytes++;
				lpc22xx_pca_rx_ack(drv_data, numbytes < msg->len - 1);
				break;
			}
			curmsg++; numbytes = 0;
			if (curmsg == num)
				lpc22xx_pca_stop(drv_data);
			else
				lpc22xx_pca_repeated_start(drv_data);
			break;

		case 0x48: /* SLA+R has been transmitted; NOT ACK has been received */
#ifdef CONFIG_I2C_DEBUG_ALGO
			printk("NOT ACK received after SLA+R\n");
#endif
			lpc22xx_pca_stop(drv_data);
			goto out;

		case 0x30: /* Data byte in I2CDAT has been transmitted; NOT ACK has been received */
#ifdef CONFIG_I2C_DEBUG_ALGO
			printk("NOT ACK received after data byte\n");
#endif
			lpc22xx_pca_stop(drv_data);
			goto out;

		case 0x38: /* Arbitration lost during SLA+W, SLA+R or data bytes */
#ifdef CONFIG_I2C_DEBUG_ALGO
			printk("Arbitration lost\n");
#endif
			lpc22xx_pca_start(drv_data);
			goto out;
			
		case 0x58: /* Data byte has been received; NOT ACK has been returned */
			if ( numbytes == msg->len - 1 ) {
				lpc22xx_pca_rx_byte(drv_data, &msg->buf[numbytes], 0);
				curmsg++; numbytes = 0;
				if (curmsg == num)
					lpc22xx_pca_stop(drv_data);
				else
					lpc22xx_pca_repeated_start(drv_data);
			} else {
#ifdef CONFIG_I2C_DEBUG_ALGO
				printk("NOT ACK sent after data byte received. "
				     "Not final byte. numbytes %d. len %d\n",
				     numbytes, msg->len);
#endif
				lpc22xx_pca_stop(drv_data);
				goto out;
			}
			break;
		case 0x70: /* Bus error - SDA stuck low */
#ifdef CONFIG_I2C_DEBUG_ALGO
			printk("BUS ERROR - SDA Stuck low\n");
#endif
			lpc22xx_pca_reset(drv_data);
			goto out;
		case 0x90: /* Bus error - SCL stuck low */
#ifdef CONFIG_I2C_DEBUG_ALGO
			printk("BUS ERROR - SCL Stuck low\n");
#endif
			lpc22xx_pca_reset(drv_data);
			goto out;
		case 0x00: /* Bus error during master or slave mode due to illegal START or STOP condition */
#ifdef CONFIG_I2C_DEBUG_ALGO
			printk("BUS ERROR - Illegal START or STOP\n");
#endif
			lpc22xx_pca_reset(drv_data);
			goto out;
		default:
			printk(KERN_ERR DRIVER ": unhandled SIO state 0x%02x\n", state);
			break;
		}
		
	}

	ret = curmsg;
 out:
#ifdef CONFIG_I2C_DEBUG_ALGO
	printk(KERN_CRIT "}}} transfered %d/%d messages. "
	     "status is %#04x. control is %#04x\n", 
	     curmsg, num, (int)lpc22xx_pca_status(drv_data),
	     (int)lpc22xx_pca_get_con(drv_data));
#endif
	return ret;
}

static u32 lpc22xx_pca_func(struct i2c_adapter *i2c_adap)
{
        return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm lpc22xx_i2c_algo = {
	.master_xfer	= lpc22xx_pca_xfer,
	.functionality	= lpc22xx_pca_func,
};

/*
 * I2C interrupt handler
 */
static irqreturn_t lpc22xx_i2c_interrupt(int irq, void *dev_id)
{
	struct lpc22xx_i2c_data *drv_data = dev_id;
	wake_up_interruptible(&drv_data->wait);
	return IRQ_HANDLED;
}

/*
 * Initialize and install I2C driver
 */
static int __init lpc22xx_i2c_probe(struct platform_device *pdev)
{
	struct lpc22xx_i2c_data *drv_data;
	struct resource *res;
	unsigned long base;
	int irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;
	base = res->start;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res)
		irq = -1;
	else
		irq = res->start;
#ifdef CONFIG_I2C_DEBUG_BUS
	printk("%s base=%p irq=%d\n", __FUNCTION__, (void *)base, irq);
#endif

	drv_data = kzalloc(sizeof(struct lpc22xx_i2c_data), GFP_KERNEL);
	if (drv_data == NULL) {
		printk(KERN_ERR DRIVER ": Can't allocate driver data!\n");
		return -ENODEV;
	}
	drv_data->base = (struct lpc22xx_i2c_regs *)base;
	drv_data->irq = irq;
	if (irq > -1) {
		if (request_irq(irq, lpc22xx_i2c_interrupt, 0, DRIVER, drv_data) < 0) {
			kfree(drv_data);
			printk(KERN_ERR DRIVER ": Request IRQ %d fails.\n", irq);
			return -ENODEV;
		}
	}
	strncpy(drv_data->adapter.name, "lpc22xx-i2c adapter", sizeof(drv_data->adapter.name));
	drv_data->adapter.id = I2C_HW_A_LPC22XX;
	drv_data->adapter.owner = THIS_MODULE;
	drv_data->adapter.class = I2C_CLASS_HWMON;
	drv_data->adapter.dev.parent = &pdev->dev;

	drv_data->adapter.algo = &lpc22xx_i2c_algo;
	drv_data->adapter.algo_data = drv_data;

	drv_data->adapter.timeout = 100;		/* default values, should	*/
	drv_data->adapter.retries = 3;			/* be replaced by defines	*/

	if (lpc22xx_pca_init(drv_data) < 0) {
		if (irq > -1) free_irq(irq, drv_data);
		kfree(drv_data);
		printk(KERN_ERR DRIVER ": I2C init fails.\n");
		return -ENODEV;
	}

	if(i2c_add_adapter(&drv_data->adapter) < 0) {
		if (irq > -1) free_irq(irq, drv_data);
		kfree(drv_data);
		printk(KERN_ERR DRIVER ": Request IRQ %d fails.\n", irq);
		return -ENODEV;
	}

	platform_set_drvdata(pdev, drv_data);

	return 0;
}

/*
 * Disable and remove the I2C driver
 */
static int lpc22xx_i2c_remove(struct platform_device *pdev)
{
	struct lpc22xx_i2c_data *drv_data = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	i2c_del_adapter(&drv_data->adapter);

	if (drv_data->irq > -1) free_irq(drv_data->irq, drv_data);

	kfree(drv_data);

	return 0;
}

#define lpc22xx_i2c_suspend NULL
#define lpc22xx_i2c_resume  NULL

static struct platform_driver lpc22xx_i2c_driver = {
	.probe		= lpc22xx_i2c_probe,
	.remove		= lpc22xx_i2c_remove,
	.suspend	= lpc22xx_i2c_suspend,
	.resume		= lpc22xx_i2c_resume,
	.driver		= {
		.name	= DRIVER,
		.owner	= THIS_MODULE,
	},
};

static int __init lpc22xx_i2c_init(void)
{
	return platform_driver_register(&lpc22xx_i2c_driver);
}

static void __exit lpc22xx_i2c_exit(void)
{
	platform_driver_unregister(&lpc22xx_i2c_driver);
}

module_init(lpc22xx_i2c_init);
module_exit(lpc22xx_i2c_exit);

MODULE_AUTHOR("Siemens Building Technologies, Philippe GOETZ <mailto:philippe.goetz@siemens.com>");
MODULE_DESCRIPTION("I2C driver for NXP LPC22XX");
MODULE_LICENSE("GPL");
/* This is reserved for the future
module_param(base, ulong, 0);
MODULE_PARM_DESC(base, "IOMEM base address");
module_param(irq, int, 0);
MODULE_PARM_DESC(irq, "IRQ number");
*/
