/*
 *  linux/arch/arm/mach-lpc22xx/gpio.c
 *
 *  Author: Philippe GOETZ (philippe dot goetz at siemens dot com)
 *
 *  Copyright (C) 2007 Siemens Building Technologies
 */

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <linux/platform_device.h>

#include <linux/serial_8250.h>

#include <asm/arch/lpc22xx.h>
#include <asm/arch/irqs.h>

#define TAB_PINSEL	((volatile unsigned long *) (APB_PINSEL_BASE+0x0000))
#define TAB_PINMODE	((volatile unsigned long *) (APB_PINSEL_BASE+0x0040))

/*
 * Change the pin function and mode to its final functions (0,1,2 or 3).
 * mode = 0 => pullup
 * mode = 2 => no pullup/no pulldown
 * mode = 3 => pulldown
 * The kernel will panic, if you try to change the mode of an pin, which is not in previously in gpio mode.
 */
int __init_or_module lpc22xx_set_periph(unsigned pin, unsigned pinsel, int pinmode)
{
	unsigned reg = pin >> 4;
	unsigned shift = ((pin & 15) * 2);
	unsigned long reg_mask = 3 << shift;
	unsigned long reg_pinsel = (pinsel & 3) << shift;
	unsigned long reg_pinmode = (pinmode & 3) << shift;

	printk("%s(P%d.%d,%d,%d) =>", __FUNCTION__, pin>>5, pin&0x1F, pinsel, pinmode);
	if((TAB_PINSEL[reg] & reg_mask) == 0) {
		TAB_PINSEL[reg] = TAB_PINSEL[reg] | reg_pinsel;
		printk(" %08lx\n", reg_pinsel);
 	}
	else if((TAB_PINSEL[reg] & reg_mask) != reg_pinsel) {
		printk(" already used in mode=%ld\n", (TAB_PINSEL[reg] >> shift) & 3);
	}
	else {
		printk(" already configured\n");
	}
	TAB_PINMODE[reg] = (TAB_PINMODE[reg] & ~reg_mask) | reg_pinmode;
	printk("%s %p=%08lx %p=%08lx\n", __FUNCTION__, &TAB_PINSEL[reg], TAB_PINSEL[reg], &TAB_PINMODE[reg], TAB_PINMODE[reg]);
 	return 0;
}
EXPORT_SYMBOL(lpc22xx_set_periph);
