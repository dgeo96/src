/*
 *	Linux Real Time Clock driver for NXP LPC22XX
 *
 *	Copyright (C) 2007 Siemens Building Technologies
 *	                   mailto:philippe.goetz@siemens.com
 *
 *      Based on rtc-at91rm9200.c Rick Bronson
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/completion.h>
#include <asm/hardware.h>
#include <asm/mach/time.h>
#include "rtc-lpc22xx.h"

#define LPC22XX_RTC_FREQ	1
#define LPC22XX_RTC_EPOCH	1900UL	/* just like arch/arm/common/rtctime.c */

/*
 * Read current time and date in RTC
 */
static int lpc22xx_rtc_readtime(struct device *dev, struct rtc_time *tm)
{
	unsigned long ctime0, ctime1, ctime2;

	/* must read twice in case it changes */
	do {
		ctime0 = REG_RTC_CTIME0;
		ctime1 = REG_RTC_CTIME1;
		ctime2 = REG_RTC_CTIME2;
	} while ((ctime0 != REG_RTC_CTIME0) || (ctime1 != REG_RTC_CTIME1) || (ctime2 != REG_RTC_CTIME2));

	tm->tm_sec  = ((ctime0 >>  0)) & 0x3F;
	tm->tm_min  = ((ctime0 >>  8)) & 0x3F;
	tm->tm_hour = ((ctime0 >> 16)) & 0x1F;
	tm->tm_wday = ((ctime0 >> 24)) & 0x07;	/* day of the week [0-6], Sunday=0 */

	tm->tm_mday = ((ctime1 >>  0) & 0x1F);
	tm->tm_mon  = ((ctime1 >>  8) & 0x0F) - 1;
	tm->tm_year = ((ctime1 >> 16) & 0xFFF);

	tm->tm_yday = ((ctime2 >>  0) & 0x1FF) - 1;

#ifdef CONFIG_RTC_DEBUG
	printk(KERN_INFO "%s(...,%4d-%02d-%02d %02d:%02d:%02d)\n", __FUNCTION__,
		1900 + tm->tm_year, tm->tm_mon+1, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);
#endif

	return 0;
}

/*
 * Set current time and date in RTC
 */
static int lpc22xx_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	unsigned char amr;

#ifdef CONFIG_RTC_DEBUG
	printk(KERN_INFO "%s(...,%4d-%02d-%02d %02d:%02d:%02d)\n", __FUNCTION__,
		1900 + tm->tm_year, tm->tm_mon+1, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);
#endif
	amr = REG_RTC_AMR;
	/* Stop counters - Disable alarm */
	REG_RTC_CCR = 0x02;		// CTTEST=0:0 / CTCRST=1 / CLKEN=0
	REG_RTC_AMR = 0xFF; 
	REG_RTC_SEC = tm->tm_sec;
	REG_RTC_MIN = tm->tm_min;
	REG_RTC_HOUR = tm->tm_hour;
	REG_RTC_DOM = tm->tm_mday;
	REG_RTC_DOW = tm->tm_wday;
	REG_RTC_DOY = tm->tm_yday + 1;
	REG_RTC_MONTH = tm->tm_mon + 1;
	REG_RTC_YEAR = tm->tm_year;
	/* Restart counters - Enable alarm */
#ifdef CONFIG_LPC22XX_RTC_CLKSRC
	REG_RTC_CCR = 0x11; // CLKSRC=1 / CTTEST=0:0 / CRCRST=0 / CLKEN=1
#else
	REG_RTC_PREINT = (LPC22xx_Fpclk / 32768) -1;
	REG_RTC_PREFRAC = LPC22xx_Fpclk - ((REG_RTC_PREINT +1) * 32768);
	REG_RTC_CCR = 0x01; // CLKSRC=0 / CTTEST=0:0 / CRCRST=0 / CLKEN=1
#endif
	REG_RTC_AMR = amr;

	return 0;
}

/*
 * Read alarm time and date in RTC
 */
static int lpc22xx_rtc_readalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time *tm = &alrm->time;

	tm->tm_sec  = (REG_RTC_ALSEC  & 0x3F);
	tm->tm_min  = (REG_RTC_ALMIN  & 0x3F);
	tm->tm_hour = (REG_RTC_ALHOUR & 0x1F);
	tm->tm_mday = (REG_RTC_ALDOM  & 0x1F);
	tm->tm_wday = (REG_RTC_ALDOW  & 0x07);
	tm->tm_yday = (REG_RTC_ALDOY  & 0x1FF) - 1;
	tm->tm_mon  = (REG_RTC_ALMON  & 0x0F) - 1;
	tm->tm_year = (REG_RTC_ALYEAR & 0xFFF);

	if(REG_RTC_AMR&0x01) tm->tm_sec = 60;
	if(REG_RTC_AMR&0x02) tm->tm_min = 60;
	if(REG_RTC_AMR&0x04) tm->tm_hour = 25;
	if(REG_RTC_AMR&0x08) tm->tm_mday = 32;
	if(REG_RTC_AMR&0x10) tm->tm_wday = 7;
	if(REG_RTC_AMR&0x20) tm->tm_yday = 366;
	if(REG_RTC_AMR&0x40) tm->tm_mon = 12;
	if(REG_RTC_AMR&0x80) tm->tm_year = 201;

	alrm->enabled = (REG_RTC_AMR & 0xFF)!=0xFF ? 1 : 0;

#ifdef CONFIG_RTC_DEBUG
	printk(KERN_INFO "%s(...,%4d-%02d-%02d %02d:%02d:%02d)\n", __FUNCTION__,
		1900 + tm->tm_year, tm->tm_mon+1, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);
#endif
	return 0;
}

/*
 * Set alarm time and date in RTC
 */
static int lpc22xx_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time *tm = &alrm->time;

#ifdef CONFIG_RTC_DEBUG
	printk(KERN_INFO "%s(...,%4d-%02d-%02d %02d:%02d:%02d)\n", __FUNCTION__,
		1900 + tm->tm_year, tm->tm_mon+1, tm->tm_mday, tm->tm_hour,
		tm->tm_min, tm->tm_sec);
#endif
	REG_RTC_ALSEC  = tm->tm_sec;
	REG_RTC_ALMIN  = tm->tm_min;
	REG_RTC_ALHOUR = tm->tm_hour;
	if(REG_RTC_ALHOUR==24) REG_RTC_ALHOUR = 0;
	REG_RTC_ALDOM  = tm->tm_mday;
	REG_RTC_ALDOW  = tm->tm_wday;
	REG_RTC_ALDOY  = tm->tm_yday + 1;
	REG_RTC_ALMON  = tm->tm_mon + 1;
	REG_RTC_ALYEAR = tm->tm_year;

	return 0;
}

/*
 * Handle commands from user-space
 */
static int lpc22xx_rtc_ioctl(struct device *dev, unsigned int cmd,
			unsigned long arg)
{
	int ret = 0;
#ifdef CONFIG_RTC_DEBUG
	printk(KERN_INFO "%s(0x%x, 0x%lx)", __FUNCTION__, cmd, arg);
#endif
	switch (cmd) {
	case RTC_AIE_OFF:	/* alarm off */
		REG_RTC_AMR = 0xFF;
		break;
	case RTC_AIE_ON:	/* alarm on */
	{
		unsigned char amr = 0xFF;
		if((REG_RTC_ALSEC  & 0x3F)<=59) amr &= ~0x01;
		if((REG_RTC_ALMIN  & 0x3F)<=59) amr &= ~0x02;
		if((REG_RTC_ALHOUR  & 0x1F)<=24) amr &= ~0x04;
		if((REG_RTC_ALDOM  & 0x1F)<=31) amr &= ~0x08;
		if((REG_RTC_ALDOM  & 0x1F)==0) amr |= 0x08;
		if((REG_RTC_ALMON  & 0x0F)<=12) amr &= ~0x40;
		if((REG_RTC_ALYEAR  & 0xFFF)<=200) amr &= ~0x80;
		REG_RTC_AMR = amr;
		break;
	}
	case RTC_UIE_OFF:	/* update off */
		REG_RTC_CIIR = 0x00;
		break;
	case RTC_UIE_ON:	/* update on */
		REG_RTC_CIIR = 0x01;	/* Interruption on Second value incrementation */
		break;
	case RTC_PIE_OFF:	/* periodic off */
		REG_RTC_CISS = REG_RTC_CISS & 0x07;
		break;
	case RTC_PIE_ON:	/* periodic on */
		REG_RTC_CISS = (REG_RTC_CISS & 0x07) | 0x80;
		break;
	case RTC_IRQP_READ:	/* read periodic alarm frequency */
		ret = put_user(2048L >> (REG_RTC_CISS & 0x07), (unsigned long *) arg);
		break;
	case RTC_IRQP_SET:	/* set periodic alarm frequency 2048,1024,512,256,128,64,32,16 Hz */
		{
			unsigned long freq = 2048L;
			unsigned char ciss;
			for(ciss = 0; ciss < 8; ciss++) {
				if(arg==freq) break;
				freq >>= 1;
			}
			if (ciss != 8)
				REG_RTC_CISS = (REG_RTC_CISS & 0x80) | ciss;
			else
				ret = -EINVAL;
		}
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
#ifdef CONFIG_RTC_DEBUG
	printk("=0x%x\n", __FUNCTION__, ret);
#endif
	return ret;
}

/*
 * Provide additional RTC information in /proc/driver/rtc
 */
static int lpc22xx_rtc_proc(struct device *dev, struct seq_file *seq)
{
#ifdef CONFIG_RTC_DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	// ILR & 0x01 RTCCIF
	seq_printf(seq, "update_IRQ\t: %s\n",
			(REG_RTC_CIIR & 0xFF) ? "yes" : "no");
	// ILR & 0x02 RTCALF
	seq_printf(seq, "alarm_IRQ\t: %s\n",
			(REG_RTC_AMR & 0xFF)!=0xFF ? "yes" : "no");
	// ILR & 0x03 RTSSF
	seq_printf(seq, "periodic_IRQ\t: %s\n",
			(REG_RTC_CISS & 0x80) ? "yes" : "no");
	seq_printf(seq, "periodic_frequency\t: %ld Hz\n",
			(unsigned long)2048L >> (REG_RTC_CISS & 0x07));

	return 0;
}

/*
 * IRQ handler for the RTC
 */
static irqreturn_t lpc22xx_rtc_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	unsigned long events = 0;

	unsigned char ilr = REG_RTC_ILR & 0x07;

#ifdef CONFIG_RTC_DEBUG
	printk(KERN_INFO "%s(%d) irl=%x\n", __FUNCTION__, ilr);
#endif
	if (ilr) {		/* this interrupt is shared!  Is it ours? */
		if (ilr & 0x01) { // RTCCIF counter increment interrupt
			events |= (RTC_UF | RTC_IRQF);
		}
		if (ilr & 0x02) { // RTCALF alarm interrupt
			events |= (RTC_AF | RTC_IRQF);
		}
		if (ilr & 0x04) { // RTCSF sub-second interrupt
			events |= (RTC_PF | RTC_IRQF);
		}
		REG_RTC_ILR = ilr;

		rtc_update_irq(&rtc->class_dev, 1, events);

#ifdef CONFIG_RTC_DEBUG
		printk("%s(): num=%ld, events=0x%02lx\n", __FUNCTION__,
			events >> 8, events & 0x000000FF);
#endif
		return IRQ_HANDLED;
	}
	return IRQ_NONE;		/* not handled */
}

static const struct rtc_class_ops lpc22xx_rtc_ops = {
	.ioctl		= lpc22xx_rtc_ioctl,
	.read_time	= lpc22xx_rtc_readtime,
	.set_time	= lpc22xx_rtc_settime,
	.read_alarm	= lpc22xx_rtc_readalarm,
	.set_alarm	= lpc22xx_rtc_setalarm,
	.proc		= lpc22xx_rtc_proc,
};

/*
 * Initialize and install RTC driver
 */
static int __init lpc22xx_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;
	int ret;

#ifdef CONFIG_RTC_DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	/* Disable all interrupts */
	REG_RTC_CIIR = 0x00;	// Disable counter increment interrupt
	REG_RTC_AMR = 0xFF;	// Disable alarm interrupt
	REG_RTC_CISS = 0x00; 	// Disable sub-second interrupt
	REG_RTC_ILR = 0x07;	// Clear any pending interrupts
	/* Ensure that the RTC is running */
#ifdef CONFIG_LPC22XX_RTC_CLKSRC
	REG_RTC_CCR = 0x11; // CLKSRC=1 / CTTEST=0:0 / CRCRST=0 / CLKEN=1
#else
	REG_RTC_PREINT = (LPC22xx_Fpclk / 32768) -1;
	REG_RTC_PREFRAC = LPC22xx_Fpclk - ((REG_RTC_PREINT +1) * 32768);
	REG_RTC_CCR = 0x01; // CLKSRC=0 / CTTEST=0:0 / CRCRST=0 / CLKEN=1
#endif

	ret = request_irq(LPC22xx_INTERRUPT_RTC, lpc22xx_rtc_interrupt,
				0,
				"lpc22xx-rtc", pdev);
	if (ret) {
		printk(KERN_ERR "lpc22xx-rtc: Request IRQ %d fails.\n",
				LPC22xx_INTERRUPT_RTC);
		return ret;
	}

	if (!device_can_wakeup(&pdev->dev))
		device_init_wakeup(&pdev->dev, 1);

	rtc = rtc_device_register(pdev->name, &pdev->dev,
				&lpc22xx_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		free_irq(LPC22xx_INTERRUPT_RTC, pdev);
		return PTR_ERR(rtc);
	}
	platform_set_drvdata(pdev, rtc);

	printk(KERN_INFO "lpc22xx-rtc: RTC driver for NXP LPC22XX.\n");

	return 0;
}

/*
 * Disable and remove the RTC driver
 */
static int lpc22xx_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);

#ifdef CONFIG_RTC_DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	/* Disable all interrupts */
	REG_RTC_CIIR = 0x00;	// Disable counter increment interrupt
	REG_RTC_AMR = 0xFF;	// Disable alarm interrupt
	REG_RTC_CISS = 0x00; 	// Disable sub-second interrupt
	REG_RTC_ILR = 0x07;	// Clear any pending interrupts

	free_irq(LPC22xx_INTERRUPT_RTC, pdev);

	rtc_device_unregister(rtc);
	platform_set_drvdata(pdev, NULL);
	device_init_wakeup(&pdev->dev, 0);

	return 0;
}


#ifdef CONFIG_PM
static struct timespec lpc22xx_rtc_delta;

static int lpc22xx_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct rtc_time tm;
	struct timespec time;

	time.tv_nsec = 0;

	/* calculate time delta for suspend */
	lpc22xx_rtc_readtime(&pdev->dev, &tm);
	rtc_tm_to_time(&tm, &time.tv_sec);
	save_time_delta(&lpc22xx_rtc_delta, &time);

	if((REG_RTC_CIIR & 0xFF)||
	   ((REG_RTC_AMR & 0xFF)!=0xFF)||
	   (REG_RTC_CISS & 0x80)) {
		if (device_may_wakeup(&pdev->dev))
			enable_irq_wake(LPC22xx_INTERRUPT_RTC);
	}

	return 0;
}

static int lpc22xx_rtc_resume(struct platform_device *pdev)
{
	struct rtc_time tm;
	struct timespec time;

	time.tv_nsec = 0;

	lpc22xx_rtc_readtime(&pdev->dev, &tm);
	rtc_tm_to_time(&tm, &time.tv_sec);
	restore_time_delta(&lpc22xx_rtc_delta, &time);

#ifdef CONFIG_RTC_DEBUG
	printk("%s(): %4d-%02d-%02d %02d:%02d:%02d\n", __FUNCTION__,
		1900 + tm.tm_year, tm.tm_mon, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec);
#endif
	return 0;
}
#else
#define lpc22xx_rtc_suspend NULL
#define lpc22xx_rtc_resume  NULL
#endif
static struct platform_driver lpc22xx_rtc_driver = {
	.probe		= lpc22xx_rtc_probe,
	.remove		= lpc22xx_rtc_remove,
	.suspend	= lpc22xx_rtc_suspend,
	.resume		= lpc22xx_rtc_resume,
	.driver		= {
		.name	= "lpc22xx-rtc",
		.owner	= THIS_MODULE,
	},
};

static int __init lpc22xx_rtc_init(void)
{
#ifdef CONFIG_RTC_DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	return platform_driver_register(&lpc22xx_rtc_driver);
}

static void __exit lpc22xx_rtc_exit(void)
{
#ifdef CONFIG_RTC_DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	platform_driver_unregister(&lpc22xx_rtc_driver);
}

module_init(lpc22xx_rtc_init);
module_exit(lpc22xx_rtc_exit);

MODULE_AUTHOR("Siemens Building Technologies, Philippe GOETZ <mailto:philippe.goetz@siemens.com>");
MODULE_DESCRIPTION("RTC driver for NXP LPC22XX");
MODULE_LICENSE("GPL");
