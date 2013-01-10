/*
 *  linux/arch/arm/mach-lpc22xx/lpc2468_devices.c
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

/* --------------------------------------------------------------------
 *  UARTs
 * -------------------------------------------------------------------- */
static struct resource lpc22xx_uart0_resource = {
	.start	= REG_UART0_BASE,
	.end	= REG_UART0_BASE + REG_UART0_SIZE - 1,
	.flags	= IORESOURCE_MEM,
};
static struct plat_serial8250_port lpc22xx_uart0_data[] = {
	{
		.mapbase	= REG_UART0_BASE,
		.membase 	= (char*)REG_UART0_BASE,
		.irq		= LPC22xx_INTERRUPT_UART0,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= LPC22xx_Fcclk,
	},
	{ }
};

static struct platform_device lpc22xx_uart0_device = {
	.name		= "serial8250",
	.id		= PLAT8250_DEV_PLATFORM,
	.dev			= {
		.platform_data	= lpc22xx_uart0_data,
	},
	.num_resources	= 1,
	.resource	= &lpc22xx_uart0_resource,
};

static void __init lpc22xx_add_device_uart0(void)
{
	platform_device_register(&lpc22xx_uart0_device);
}

/* --------------------------------------------------------------------
 *  RTC
 * -------------------------------------------------------------------- */
#if defined(CONFIG_RTC_DRV_LPC22XX) || defined(CONFIG_RTC_DRV_LPC22XX_MODULE)
/*
static struct resource lpc22xx_rtc_resource = {
	{
		.start	= REG_RTC_BASE,
		.end	= REG_RTC_BASE + REG_RTC_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	}
};
*/
static struct platform_device lpc22xx_rtc_device = {
	.name		= "lpc22xx-rtc",
	.id		= -1,
	.num_resources	= 0,
};

static void __init lpc22xx_add_device_rtc(void)
{
	platform_device_register(&lpc22xx_rtc_device);
}
#else
static void __init lpc22xx_add_device_rtc(void) {}
#endif

/* -------------------------------------------------------------------- */

/*
 * These devices are always present and don't need any board-specific
 * setup.
 */
static int __init lpc22xx_add_standard_devices(void)
{
	lpc22xx_add_device_uart0();
//	lpc22xx_add_device_rtc();
	return 0;
}

arch_initcall(lpc22xx_add_standard_devices);
