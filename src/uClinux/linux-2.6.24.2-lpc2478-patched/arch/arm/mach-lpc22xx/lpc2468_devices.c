/*
 *  linux/arch/arm/mach-lpc22xx/lpc2468_devices.c
 *
 *  Author: Philippe GOETZ (philippe dot goetz at siemens dot com)
 *
 *  Copyright (C) 2007 Siemens Building Technologies
 *
 * This file is used to define the embedded peripherals as platform devices.
 *
 * This will also setup for each platform devices as required:
 * - GPIO
 * - CLK
 * - PWR
 * Memory and Interrupt are defined using resource structures.
 *
 */

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <linux/platform_device.h>

#include <linux/serial_8250.h>

#include <asm/arch/lpc22xx.h>
#include <asm/arch/irqs.h>
#include <asm/arch/gpio.h>

/* Additional registers and parameters */
#define REG_CPC_PCONP              (*((volatile unsigned long *) (0xE01FC0C4)))
#define KREG_CPC_PCONP_PCENET        0x40000000L

/* --------------------------------------------------------------------
 *  VIC & EMC & GPDMA
 * -------------------------------------------------------------------- */
static struct resource lpc22xx_global_resource[] = {
	{
		.name	= "lpc22xx-vic",
		.start	= APH_VIC_BASE,
		.end	= APH_VIC_BASE + APH_VIC_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "lpc22xx-emc",
		.start	= APH_EMC_BASE,
		.end	= APH_EMC_BASE + APH_EMC_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "lpc22xx-gpdma",
		.start	= APH_GPDMA_BASE,
		.end	= APH_GPDMA_BASE + APH_GPDMA_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};
static struct platform_device lpc22xx_global_device = {
	.name		= "lpc22xx-global",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(lpc22xx_global_resource),
	.resource	= lpc22xx_global_resource,
};

static void __init lpc22xx_add_global_device(void)
{
	platform_device_register(&lpc22xx_global_device);
}

/* --------------------------------------------------------------------
 *  GPIO & PINSEL
 * -------------------------------------------------------------------- */
static struct resource lpc22xx_gpio_resource[] = {
	{
		.name	= "lpc22xx-gpio",
		.start	= APB_GPIO_BASE,
		.end	= APB_GPIO_BASE + APB_GPIO_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "lpc22xx-pinsel",
		.start	= APB_PINSEL_BASE,
		.end	= APB_PINSEL_BASE + APB_PINSEL_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};
static struct platform_device lpc22xx_gpio_device = {
	.name		= "lpc22xx-gpio",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(lpc22xx_gpio_resource),
	.resource	= lpc22xx_gpio_resource,
};

static void __init lpc22xx_add_gpio_device(void)
{
	platform_device_register(&lpc22xx_gpio_device);
}

/* --------------------------------------------------------------------
 *  Internal Flash
 * -------------------------------------------------------------------- */
#ifdef CONFIG_MTD_LPC22XX_FLASH
extern struct mtd_partition lpc22xx_flash_partition_info[];

static struct resource lpc22xx_flash_resource[] = {
	{
		.name	= "lpc22xx-flash",
		.start	= INTERNAL_FLASH_BASE,
		.end	= INTERNAL_FLASH_BASE + INTERNAL_FLASH_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};
static struct platform_device lpc22xx_flash_device = {
	.name		= "lpc22xx-flash",
	.id		= -1,
#ifdef CONFIG_MTD_PARTITIONS
	.dev		= {
		.platform_data	= lpc22xx_flash_partition_info,
	},
#endif
	.num_resources	= ARRAY_SIZE(lpc22xx_flash_resource),
	.resource	= lpc22xx_flash_resource,
};

static void __init lpc22xx_add_flash_device(void)
{
	platform_device_register(&lpc22xx_flash_device);
}
#endif
/* --------------------------------------------------------------------
 *  TIMERs (Chapter ??)
 * -------------------------------------------------------------------- */
static struct resource lpc22xx_timer_resource[] = {
	{
		.name	= "lpc22xx-timer0",
		.start	= APB_TIMER0_BASE,
		.end	= APB_TIMER0_BASE + APB_TIMER0_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#if defined(CONFIG_LPC22XX_TIMER1)
	{
		.name	= "lpc22xx-timer1",
		.start	= APB_TIMER1_BASE,
		.end	= APB_TIMER1_BASE + APB_TIMER1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
#if defined(CONFIG_LPC22XX_TIMER2)
	{
		.name	= "lpc22xx-timer2",
		.start	= APB_TIMER2_BASE,
		.end	= APB_TIMER2_BASE + APB_TIMER2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
#if defined(CONFIG_LPC22XX_TIMER3)
	{
		.name	= "lpc22xx-timer3",
		.start	= APB_TIMER3_BASE,
		.end	= APB_TIMER3_BASE + APB_TIMER3_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
};

static struct platform_device lpc22xx_timer_device = {
	.name		= "lpc22xx-timer",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(lpc22xx_timer_resource),
	.resource	= lpc22xx_timer_resource,
};

static void __init lpc22xx_add_timer_device(void)
{
	platform_device_register(&lpc22xx_timer_device);
}

/* --------------------------------------------------------------------
 *  UARTs (Chapter 16 and Chapter 17)
 * -------------------------------------------------------------------- */
#if defined(CONFIG_LPC22XX_UART0) || defined(CONFIG_LPC22XX_UART1) || defined(CONFIG_LPC22XX_UART2) || defined(CONFIG_LPC22XX_UART3)
static struct resource lpc22xx_uart_resource[] = {
#if defined(CONFIG_LPC22XX_UART0)
	{
		.name	= "lpc22xx-uart0",
		.start	= APB_UART0_BASE,
		.end	= APB_UART0_BASE + APB_UART0_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
#if defined(CONFIG_LPC22XX_UART1)
	{
		.name	= "lpc22xx-uart1",
		.start	= APB_UART1_BASE,
		.end	= APB_UART1_BASE + APB_UART1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
#if defined(CONFIG_LPC22XX_UART2)
	{
		.name	= "lpc22xx-uart2",
		.start	= APB_UART2_BASE,
		.end	= APB_UART2_BASE + APB_UART2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
#if defined(CONFIG_LPC22XX_UART3)
	{
		.name	= "lpc22xx-uart3",
		.start	= APB_UART3_BASE,
		.end	= APB_UART3_BASE + APB_UART3_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
};
static struct plat_serial8250_port lpc22xx_uart_data[] = {
#if defined(CONFIG_LPC22XX_UART0)
	{
		.mapbase	= APB_UART0_BASE,
		.membase 	= (char*)APB_UART0_BASE,
		.irq		= LPC22xx_INTERRUPT_UART0,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= LPC22xx_Fcclk,
	},
#endif
#if defined(CONFIG_LPC22XX_UART1)
	{
		.mapbase	= APB_UART1_BASE,
		.membase 	= (char*)APB_UART1_BASE,
		.irq		= LPC22xx_INTERRUPT_UART1,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= LPC22xx_Fcclk,
	},
#endif
#if defined(CONFIG_LPC22XX_UART2)
	{
		.mapbase	= APB_UART2_BASE,
		.membase 	= (char*)APB_UART2_BASE,
		.irq		= LPC22xx_INTERRUPT_UART2,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= LPC22xx_Fcclk,
	},
#endif
#if defined(CONFIG_LPC22XX_UART3)
	{
		.mapbase	= APB_UART3_BASE,
		.membase 	= (char*)APB_UART3_BASE,
		.irq		= LPC22xx_INTERRUPT_UART3,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= LPC22xx_Fcclk,
	},
#endif
	{ }
};

static struct platform_device lpc22xx_uart_device = {
	.name		= "serial8250",
	.id		= PLAT8250_DEV_PLATFORM,
	.dev			= {
		.platform_data	= lpc22xx_uart_data,
	},
	.num_resources	= ARRAY_SIZE(lpc22xx_uart_resource),
	.resource	= lpc22xx_uart_resource,
};

static void __init lpc22xx_add_uart_device(void)
{
#ifdef CONFIG_LPC22XX_UART0
#ifdef CONFIG_LPC2468_PINSEL_P0_2_TXD0
	lpc22xx_set_periph(LPC22XX_PIN_P0_2, 1, 0);	/* TXD0 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_3_RXD0
	lpc22xx_set_periph(LPC22XX_PIN_P0_3, 1, 0);	/* RXD0 */
#endif
#endif

#ifdef CONFIG_LPC22XX_UART1
#ifdef CONFIG_LPC2468_PINSEL_P0_15_TXD1
	lpc22xx_set_periph(LPC22XX_PIN_P0_15, 1, 0);	/* TXD1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_0_TXD1
	lpc22xx_set_periph(LPC22XX_PIN_P2_0, 2, 0);	/* TXD1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P3_16_TXD1
	lpc22xx_set_periph(LPC22XX_PIN_P3_16, 3, 0);	/* TXD1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_0_RXD1
	lpc22xx_set_periph(LPC22XX_PIN_P0_16, 1, 0);	/* RXD1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_1_RXD1
	lpc22xx_set_periph(LPC22XX_PIN_P2_1, 2, 0);	/* RXD1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P3_17_RXD1
	lpc22xx_set_periph(LPC22XX_PIN_P3_17, 3, 0);	/* RXD1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_17_CTS1
	lpc22xx_set_periph(LPC22XX_PIN_P0_17, 1, 0);	/* CTS1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_2_CTS1
	lpc22xx_set_periph(LPC22XX_PIN_P2_2, 2, 0);	/* CTS1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P3_18_CTS1
	lpc22xx_set_periph(LPC22XX_PIN_P3_18, 3, 0);	/* CTS1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_18_DCD1
	lpc22xx_set_periph(LPC22XX_PIN_P0_18, 1, 0);	/* DCD1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_3_DCD1
	lpc22xx_set_periph(LPC22XX_PIN_P2_3, 2, 0);	/* DCD1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P3_19_DCD1
	lpc22xx_set_periph(LPC22XX_PIN_P3_19, 3, 0);	/* DCD1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_19_DSR1
	lpc22xx_set_periph(LPC22XX_PIN_P0_19, 1, 0);	/* DSR1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_4_DSR1
	lpc22xx_set_periph(LPC22XX_PIN_P2_4, 2, 0);	/* DSR1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P3_20_DSR1
	lpc22xx_set_periph(LPC22XX_PIN_P3_20, 3, 0);	/* DSR1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_20_DTR1
	lpc22xx_set_periph(LPC22XX_PIN_P0_20, 1, 0);	/* DTR1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_5_DTR1
	lpc22xx_set_periph(LPC22XX_PIN_P2_5, 2, 0);	/* DTR1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P3_21_DTR1
	lpc22xx_set_periph(LPC22XX_PIN_P3_21, 3, 0);	/* DTR1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_21_RI1
	lpc22xx_set_periph(LPC22XX_PIN_P0_21, 1, 0);	/* RI1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_6_RI1
	lpc22xx_set_periph(LPC22XX_PIN_P2_6, 2, 0);	/* RI1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P3_22_RI1
	lpc22xx_set_periph(LPC22XX_PIN_P3_22, 3, 0);	/* RI1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_22_RTS1
	lpc22xx_set_periph(LPC22XX_PIN_P0_22, 1, 0);	/* RTS1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_7_RTS1
	lpc22xx_set_periph(LPC22XX_PIN_P2_7, 2, 0);	/* RTS1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P3_30_RTS1
	lpc22xx_set_periph(LPC22XX_PIN_P3_30, 3, 0);	/* RTS1 */
#endif
#endif

#if defined(CONFIG_LPC22XX_UART2)
#ifdef CONFIG_LPC2468_PINSEL_P0_10_TXD2
	lpc22xx_set_periph(LPC22XX_PIN_P0_10, 1, 0);	/* TXD2 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_8_TXD2
	lpc22xx_set_periph(LPC22XX_PIN_P2_8, 2, 0);	/* TXD2 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P4_22_TXD2
	lpc22xx_set_periph(LPC22XX_PIN_P4_22, 2, 0);	/* TXD2 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_11_RXD2
	lpc22xx_set_periph(LPC22XX_PIN_P0_11, 1, 0);	/* RXD2 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_9_RXD2
	lpc22xx_set_periph(LPC22XX_PIN_P2_9, 2, 0);	/* RXD2 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P4_23_RXD2
	lpc22xx_set_periph(LPC22XX_PIN_P4_23, 2, 0);	/* RXD2 */
#endif
#endif

#if defined(CONFIG_LPC22XX_UART3)
#ifdef CONFIG_LPC2468_PINSEL_P0_0_TXD3
	lpc22xx_set_periph(LPC22XX_PIN_P0_0, 2, 0);	/* TXD3 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_25_TXD3
	lpc22xx_set_periph(LPC22XX_PIN_P0_25, 2, 0);	/* TXD3 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P4_28_TXD3
	lpc22xx_set_periph(LPC22XX_PIN_P4_28, 3, 0);	/* TXD3 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_1_RXD3
	lpc22xx_set_periph(LPC22XX_PIN_P0_1, 2, 0);	/* RXD3 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_26_RXD3
	lpc22xx_set_periph(LPC22XX_PIN_P0_26, 2, 0);	/* RXD3 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P4_29_RXD3
	lpc22xx_set_periph(LPC22XX_PIN_P4_29, 3, 0);	/* RXD3 */
#endif
#endif
	platform_device_register(&lpc22xx_uart_device);
}
#endif

/* --------------------------------------------------------------------
 *  RTC (Chapter 28)
 * -------------------------------------------------------------------- */
#if defined(CONFIG_LPC22XX_RTC)
static struct resource lpc22xx_rtc_resource = {
	.name	= "lpc22xx-rtc",
	.start	= APB_RTC_BASE,
	.end	= APB_RTC_BASE + APB_RTC_SIZE - 1,
	.flags	= IORESOURCE_MEM,
};
static struct platform_device lpc22xx_rtc_device = {
	.name		= "lpc22xx-rtc",
	.id		= -1,
	.num_resources	= 1,
	.resource	= &lpc22xx_rtc_resource,
};

static void __init lpc22xx_add_rtc_device(void)
{
	platform_device_register(&lpc22xx_rtc_device);
}
#endif

/* --------------------------------------------------------------------
 *  Backuped SRAM (Chapter 28)
 * -------------------------------------------------------------------- */
#if defined(CONFIG_LPC22XX_NVRAM)
static struct resource lpc22xx_bram_resource = {
	.name	= "lpc22xx-nvram",
	.start	= APB_BRAM_BASE,
	.end	= APB_BRAM_BASE + APB_BRAM_SIZE - 1,
	.flags	= IORESOURCE_MEM,
};
static struct platform_device lpc22xx_bram_device = {
	.name		= "nvram",
	.id		= -1,
	.num_resources	= 1,
	.resource	= &lpc22xx_bram_resource,
};

static void __init lpc22xx_add_bram_device(void)
{
	platform_device_register(&lpc22xx_bram_device);
}

unsigned char nvram_read_byte(int i) {
	int index = i>>2;
	int shift = (i&3)*8;
	unsigned long ret = 0xFFFFFFFFL;

	if(i<APB_BRAM_ACTUAL_SIZE) {
		ret = ((volatile unsigned long *)APB_BRAM_BASE)[index];
		ret >>= shift;
	}
	return (unsigned char)ret;
}
EXPORT_SYMBOL(nvram_read_byte);
void nvram_write_byte(unsigned char c, int i) {
	int index = i>>2;
	int shift = (i&3)*8;
	if(i<APB_BRAM_ACTUAL_SIZE) {
		unsigned long ret = ((volatile unsigned long *)APB_BRAM_BASE)[index];
		ret = (ret & ~(0xFF<<shift)) | (c<<shift);
		((volatile unsigned long *)APB_BRAM_BASE)[index] = ret;
	}
}
EXPORT_SYMBOL(nvram_write_byte);
void	nvram_sync(void) {
}
EXPORT_SYMBOL(nvram_sync);
#endif

/* --------------------------------------------------------------------
 *  I2Cs (Chapter ??)
 * -------------------------------------------------------------------- */
#if defined(CONFIG_LPC22XX_I2C0)
static struct resource lpc22xx_i2c0_resource[] = {
	{
		.name	= "lpc22xx-i2c0",
		.start	= APB_I2C0_BASE,
		.end	= APB_I2C0_BASE + APB_I2C0_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "lpc22xx-i2c0",
		.start	= LPC22xx_INTERRUPT_I2C0,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device lpc22xx_i2c0_device = {
	.name		= "lpc22xx-i2c",
	.id		= PLAT8250_DEV_PLATFORM,
	.num_resources	= ARRAY_SIZE(lpc22xx_i2c0_resource),
	.resource	= lpc22xx_i2c0_resource,
};

static void __init lpc22xx_add_i2c0_device(void)
{
#ifdef CONFIG_LPC2468_PINSEL_P0_27_SCL0
	lpc22xx_set_periph(LPC22XX_PIN_P0_2, 1, 0);	/* SCL0 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_28_SDA0
	lpc22xx_set_periph(LPC22XX_PIN_P0_3, 1, 0);	/* SDA0 */
#endif
	platform_device_register(&lpc22xx_i2c0_device);
}
#endif

#if defined(CONFIG_LPC22XX_I2C1)
static struct resource lpc22xx_i2c1_resource[] = {
	{
		.name	= "lpc22xx-i2c1",
		.start	= APB_I2C1_BASE,
		.end	= APB_I2C1_BASE + APB_I2C1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "lpc22xx-i2c1",
		.start	= LPC22xx_INTERRUPT_I2C1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device lpc22xx_i2c1_device = {
	.name		= "lpc22xx-i2c",
	.id		= PLAT8250_DEV_PLATFORM,
	.num_resources	= ARRAY_SIZE(lpc22xx_i2c1_resource),
	.resource	= lpc22xx_i2c1_resource,
};

static void __init lpc22xx_add_i2c1_device(void)
{
#ifdef CONFIG_LPC2468_PINSEL_P0_0_SDA1
	lpc22xx_set_periph(LPC22XX_PIN_P0_0, 3, 0);	/* SDA1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_19_SDA1
	lpc22xx_set_periph(LPC22XX_PIN_P0_19, 3, 0);	/* SDA1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_14_SDA1
	lpc22xx_set_periph(LPC22XX_PIN_P2_14, 3, 0);	/* SDA1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_1_SCL1
	lpc22xx_set_periph(LPC22XX_PIN_P0_1, 3, 0);	/* SCL1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_20_SCL1
	lpc22xx_set_periph(LPC22XX_PIN_P0_20, 3, 0);	/* SCL1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_15_SCL1
	lpc22xx_set_periph(LPC22XX_PIN_P2_15, 3, 0);	/* SCL1 */
#endif
	platform_device_register(&lpc22xx_i2c1_device);
}
#endif

#if defined(CONFIG_LPC22XX_I2C2)
static struct resource lpc22xx_i2c2_resource[] = {
	{
		.name	= "lpc22xx-i2c2",
		.start	= APB_I2C2_BASE,
		.end	= APB_I2C2_BASE + APB_I2C2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "lpc22xx-i2c2",
		.start	= LPC22xx_INTERRUPT_I2C2,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device lpc22xx_i2c2_device = {
	.name		= "lpc22xx-i2c",
	.id		= PLAT8250_DEV_PLATFORM,
	.num_resources	= ARRAY_SIZE(lpc22xx_i2c2_resource),
	.resource	= lpc22xx_i2c2_resource,
};

static void __init lpc22xx_add_i2c2_device(void)
{
#ifdef CONFIG_LPC2468_PINSEL_P0_10_SDA2
	lpc22xx_set_periph(LPC22XX_PIN_P0_0, 1, 0);	/* SDA2 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_30_SDA2
	lpc22xx_set_periph(LPC22XX_PIN_P2_30, 3, 0);	/* SDA2 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P4_20_SDA2
	lpc22xx_set_periph(LPC22XX_PIN_P4_20, 3, 0);	/* SDA2 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_11_SCL2
	lpc22xx_set_periph(LPC22XX_PIN_P0_11, 1, 0);	/* SCL2 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_31_SCL2
	lpc22xx_set_periph(LPC22XX_PIN_P2_31, 3, 0);	/* SCL2 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P4_21_SCL2
	lpc22xx_set_periph(LPC22XX_PIN_P4_21, 3, 0);	/* SCL2 */
#endif
	platform_device_register(&lpc22xx_i2c2_device);
}
#endif

/* --------------------------------------------------------------------
 *  SPI (Chapter 18)
 * -------------------------------------------------------------------- */
#if defined(CONFIG_LPC22XX_SPI)
static struct resource lpc22xx_spi_resource = {
	.name	= "lpc22xx-spi",
	.start	= APB_SPI_BASE,
	.end	= APB_SPI_BASE + APB_SPI_SIZE - 1,
	.flags	= IORESOURCE_MEM,
};
static struct platform_device lpc22xx_spi_device = {
	.name		= "lpc22xx-spi",
	.id		= -1,
	.num_resources	= 1,
	.resource	= &lpc22xx_spi_resource,
};

static void __init lpc22xx_add_spi_device(void)
{
#ifdef CONFIG_LPC2468_PINSEL_P0_16_SSEL
	lpc22xx_set_periph(LPC22XX_PIN_P0_16, 3, 0);	/* SSEL */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_15_SCK
	lpc22xx_set_periph(LPC22XX_PIN_P0_15, 3, 0);	/* SCK */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_17_MISO
	lpc22xx_set_periph(LPC22XX_PIN_P0_17, 3, 0);	/* MISO */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_18_MOSI
	lpc22xx_set_periph(LPC22XX_PIN_P0_18, 3, 0);	/* MOSI */
#endif
	platform_device_register(&lpc22xx_spi_device);
}
#endif

/* --------------------------------------------------------------------
 *  SSPs (Chapter 19)
 * -------------------------------------------------------------------- */
#if defined(CONFIG_LPC22XX_SSP0) || defined(CONFIG_LPC22XX_SSP1)
static struct resource lpc22xx_ssp_resource[] = {
#if defined(CONFIG_LPC22XX_SSP0)
	{
		.name	= "lpc22xx-ssp0",
		.start	= APB_SSP0_BASE,
		.end	= APB_SSP0_BASE + APB_SSP0_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
#if defined(CONFIG_LPC22XX_SSP1)
	{
		.name	= "lpc22xx-ssp1",
		.start	= APB_SSP1_BASE,
		.end	= APB_SSP1_BASE + APB_SSP1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
};

static struct platform_device lpc22xx_ssp_device = {
	.name		= "lpc22xx-ssp",
	.id		= PLAT8250_DEV_PLATFORM,
	.num_resources	= ARRAY_SIZE(lpc22xx_ssp_resource),
	.resource	= lpc22xx_ssp_resource,
};

static void __init lpc22xx_add_ssp_device(void)
{
#ifdef CONFIG_LPC22XX_SSP0
#ifdef CONFIG_LPC2468_PINSEL_P0_16_SSEL0
	lpc22xx_set_periph(LPC22XX_PIN_P0_16, 2, 0);	/* SSEL0 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_21_SSEL0
	lpc22xx_set_periph(LPC22XX_PIN_P1_21, 3, 0);	/* SSEL0 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_23_SSEL0
	lpc22xx_set_periph(LPC22XX_PIN_P2_23, 3, 0);	/* SSEL0 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_15_SCK0
	lpc22xx_set_periph(LPC22XX_PIN_P0_15, 2, 0);	/* SCK0 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_20_SCK0
	lpc22xx_set_periph(LPC22XX_PIN_P1_20, 3, 0);	/* SCK0 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_22_SCK0
	lpc22xx_set_periph(LPC22XX_PIN_P2_22, 3, 0);	/* SCK0 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_17_MISO0
	lpc22xx_set_periph(LPC22XX_PIN_P0_17, 2, 0);	/* MISO0 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_23_MISO0
	lpc22xx_set_periph(LPC22XX_PIN_P1_23, 3, 0);	/* MISO0 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_26_MISO0
	lpc22xx_set_periph(LPC22XX_PIN_P2_26, 3, 0);	/* MISO0 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_18_MOSI0
	lpc22xx_set_periph(LPC22XX_PIN_P0_18, 2, 0);	/* MOSI0 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_24_MOSI0
	lpc22xx_set_periph(LPC22XX_PIN_P1_24, 3, 0);	/* MOSI0 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_27_MOSI0
	lpc22xx_set_periph(LPC22XX_PIN_P2_27, 3, 0);	/* MOSI0 */
#endif
#endif

#ifdef CONFIG_LPC22XX_SSP1
#ifdef CONFIG_LPC2468_PINSEL_P0_6_SSEL1
	lpc22xx_set_periph(LPC22XX_PIN_P0_6, 2, 1);	/* SSEL1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_14_SSEL1
	lpc22xx_set_periph(LPC22XX_PIN_P0_14, 3, 1);	/* SSEL1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P4_21_SSEL1
	lpc22xx_set_periph(LPC22XX_PIN_P4_21, 3, 1);	/* SSEL1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_7_SCK1
	lpc22xx_set_periph(LPC22XX_PIN_P0_7, 2, 1);	/* SCK1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_31_SCK1
	lpc22xx_set_periph(LPC22XX_PIN_P1_31, 2, 1);	/* SCK1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P4_20_SCK1
	lpc22xx_set_periph(LPC22XX_PIN_P4_20, 3, 1);	/* SCK1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_8_MISO1
	lpc22xx_set_periph(LPC22XX_PIN_P0_8, 2, 1);	/* MISO1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_12_MISO1
	lpc22xx_set_periph(LPC22XX_PIN_P0_12, 2, 1);	/* MISO1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P4_22_MISO1
	lpc22xx_set_periph(LPC22XX_PIN_P4_22, 3, 1);	/* MISO1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_9_MOSI1
	lpc22xx_set_periph(LPC22XX_PIN_P0_9, 2, 1);	/* MOSI1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_13_MOSI1
	lpc22xx_set_periph(LPC22XX_PIN_P0_13, 3, 1);	/* MOSI1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P4_23_MOSI1
	lpc22xx_set_periph(LPC22XX_PIN_P4_23, 3, 1);	/* MOSI1 */
#endif
#endif

	platform_device_register(&lpc22xx_ssp_device);
}
#endif

/* --------------------------------------------------------------------
 *  I2S (Chapter 22)
 * -------------------------------------------------------------------- */
#if defined(CONFIG_LPC22XX_I2S)
static struct resource lpc22xx_i2s_resource = {
	.name	= "lpc22xx-i2s",
	.start	= APB_I2S_BASE,
	.end	= APB_I2S_BASE + APB_I2S_SIZE - 1,
	.flags	= IORESOURCE_MEM,
};
static struct platform_device lpc22xx_i2s_device = {
	.name		= "lpc22xx-i2s",
	.id		= -1,
	.num_resources	= 1,
	.resource	= &lpc22xx_i2s_resource,
};

static void __init lpc22xx_add_i2s_device(void)
{
#ifdef CONFIG_LPC2468_PINSEL_P0_4_I2SRX_CLK
	lpc22xx_set_periph(LPC22XX_PIN_P0_4, 1, 0);	/* I2SRX_CLK */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_23_I2SRX_CLK
	lpc22xx_set_periph(LPC22XX_PIN_P0_23, 2, 0);	/* I2SRX_CLK */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_5_I2SRX_WS
	lpc22xx_set_periph(LPC22XX_PIN_P0_5, 1, 0);	/* I2SRX_WS */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_24_I2SRX_WS
	lpc22xx_set_periph(LPC22XX_PIN_P0_24, 2, 0);	/* I2SRX_WS */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_6_I2SRX_SDA
	lpc22xx_set_periph(LPC22XX_PIN_P0_6, 1, 0);	/* I2SRX_SDA */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_25_I2SRX_SDA
	lpc22xx_set_periph(LPC22XX_PIN_P0_25, 2, 0);	/* I2SRX_SDA */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_7_I2STX_CLK
	lpc22xx_set_periph(LPC22XX_PIN_P0_7, 1, 0);	/* I2STX_CLK */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_11_I2STX_CLK
	lpc22xx_set_periph(LPC22XX_PIN_P2_11, 3, 0);	/* I2STX_CLK */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_8_I2STX_WS
	lpc22xx_set_periph(LPC22XX_PIN_P0_8, 1, 0);	/* I2STX_WS */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_12_I2STX_WS
	lpc22xx_set_periph(LPC22XX_PIN_P2_12, 3, 0);	/* I2STX_WS */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_9_I2STX_SDA
	lpc22xx_set_periph(LPC22XX_PIN_P0_9, 1, 0);	/* I2STX_SDA */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_13_I2STX_SDA
	lpc22xx_set_periph(LPC22XX_PIN_P2_13, 3, 0);	/* I2STX_SDA */
#endif
	platform_device_register(&lpc22xx_i2s_device);
}
#endif

/* --------------------------------------------------------------------
 *  MMC (Chapter 20)
 * -------------------------------------------------------------------- */
#if defined(CONFIG_LPC22XX_MMC)
static struct resource lpc22xx_mmc_resource = {
	.name	= "lpc22xx-mmc",
	.start	= APB_MMC_BASE,
	.end	= APB_MMC_BASE + APB_MMC_SIZE - 1,
	.flags	= IORESOURCE_MEM,
};
static struct platform_device lpc22xx_mmc_device = {
	.name		= "lpc22xx-mmc",
	.id		= -1,
	.num_resources	= 1,
	.resource	= &lpc22xx_mmc_resource,
};

static void __init lpc22xx_add_mmc_device(void)
{
#ifdef CONFIG_LPC2468_PINSEL_P0_19_MCICLK
	lpc22xx_set_periph(LPC22XX_PIN_P0_19, 2, 0);	/* MCICLK */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_2_MCICLK
	lpc22xx_set_periph(LPC22XX_PIN_P1_2, 2, 0);	/* MCICLK */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_20_MCICMD
	lpc22xx_set_periph(LPC22XX_PIN_P0_20, 2, 0);	/* MCICMD */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_3_MCICMD
	lpc22xx_set_periph(LPC22XX_PIN_P1_3, 2, 0);	/* MCICMD */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_22_MCIDAT0
	lpc22xx_set_periph(LPC22XX_PIN_P0_22, 2, 0);	/* MCIDAT0 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_6_MCIDAT0
	lpc22xx_set_periph(LPC22XX_PIN_P1_6, 2, 0);	/* MCIDAT0 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_7_MCIDAT1
	lpc22xx_set_periph(LPC22XX_PIN_P1_7, 2, 0);	/* MCIDAT1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_11_MCIDAT1
	lpc22xx_set_periph(LPC22XX_PIN_P2_11, 2, 0);	/* MCIDAT1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_11_MCIDAT2
	lpc22xx_set_periph(LPC22XX_PIN_P1_11, 2, 0);	/* MCIDAT2 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_11_MCIDAT2
	lpc22xx_set_periph(LPC22XX_PIN_P2_12, 2, 0);	/* MCIDAT2 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_12_MCIDAT3
	lpc22xx_set_periph(LPC22XX_PIN_P1_12, 2, 0);	/* MCIDAT3 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_11_MCIDAT3
	lpc22xx_set_periph(LPC22XX_PIN_P2_13, 2, 0);	/* MCIDAT3 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P0_21_MCIPWR
	lpc22xx_set_periph(LPC22XX_PIN_P0_21, 2, 0);	/* MCIPWR */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_5_MCIPWR
	lpc22xx_set_periph(LPC22XX_PIN_P1_5, 2, 0);	/* MCIPWR */
#endif
	platform_device_register(&lpc22xx_mmc_device);
}
#endif

/* --------------------------------------------------------------------
 *  Ethernet (Chapter 11)
 * -------------------------------------------------------------------- */
#if defined(CONFIG_LPC22XX_ETH)
static struct resource lpc22xx_eth_resource[] = {
	{
		.name	= "lpc22xx-eth",
		.start	= APH_ETH_BASE,
		.end	= APH_ETH_BASE + APH_ETH_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "lpc22xx-eth-ram",
		.start	= ETH_RAM_BASE,
		.end	= ETH_RAM_BASE + ETH_RAM_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};
static struct platform_device lpc22xx_eth_device = {
	.name		= "lpc22xx-eth",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(lpc22xx_eth_resource),
	.resource	= lpc22xx_eth_resource,
};

static void __init lpc22xx_add_eth_device(void)
{
	// Set Ethernet Block power/clock control bit.
	REG_CPC_PCONP |= KREG_CPC_PCONP_PCENET;
	// Set pins according configuration.
#ifdef CONFIG_LPC2468_PINSEL_P1_0_ENET_TXD0
	lpc22xx_set_periph(LPC22XX_PIN_P1_0, 1, 0);	/* ENET_TXD0 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_1_ENET_TXD1
	lpc22xx_set_periph(LPC22XX_PIN_P1_1, 1, 0);	/* ENET_TXD1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_2_ENET_TXD2
	lpc22xx_set_periph(LPC22XX_PIN_P1_2, 1, 0);	/* ENET_TXD2 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_3_ENET_TXD3
	lpc22xx_set_periph(LPC22XX_PIN_P1_3, 1, 0);	/* ENET_TXD3 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_4_ENET_TX_EN
	lpc22xx_set_periph(LPC22XX_PIN_P1_4, 1, 0);	/* ENET_TX_EN */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_5_ENET_TX_ER
	lpc22xx_set_periph(LPC22XX_PIN_P1_5, 1, 0);	/* ENET_TX_ER */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_6_ENET_TX_CLK
	lpc22xx_set_periph(LPC22XX_PIN_P1_6, 1, 0);	/* ENET_TX_CLK */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_7_ENET_COL
	lpc22xx_set_periph(LPC22XX_PIN_P1_7, 1, 0);	/* ENET_COL */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_8_ENET_CRS
	lpc22xx_set_periph(LPC22XX_PIN_P1_8, 1, 0);	/* ENET_CRS or ENET_CRS_DV */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_9_ENET_RXD0
	lpc22xx_set_periph(LPC22XX_PIN_P1_9, 1, 0);	/* ENET_RXD0 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_10_ENET_RXD1
	lpc22xx_set_periph(LPC22XX_PIN_P1_10, 1, 0);	/* ENET_RXD1 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_11_ENET_RXD2
	lpc22xx_set_periph(LPC22XX_PIN_P1_11, 1, 0);	/* ENET_RXD2 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_12_ENET_RXD3
	lpc22xx_set_periph(LPC22XX_PIN_P1_12, 1, 0);	/* ENET_RXD3 */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_13_ENET_RX_DV
	lpc22xx_set_periph(LPC22XX_PIN_P1_13, 1, 0);	/* ENET_RX_DV */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_14_ENET_RX_ER
	lpc22xx_set_periph(LPC22XX_PIN_P1_14, 1, 0);	/* ENET_RX_ER */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_15_ENET_RX_CLK
	lpc22xx_set_periph(LPC22XX_PIN_P1_15, 1, 0);	/* ENET_RX_CLK or ENET_REF_CLK */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_16_ENET_MDC
	lpc22xx_set_periph(LPC22XX_PIN_P1_16, 1, 0);	/* ENET_MDC */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P1_17_ENET_MDIO
	lpc22xx_set_periph(LPC22XX_PIN_P1_17, 1, 0);	/* ENET_MDIO */
#endif
#ifdef CONFIG_LPC2468_PINSEL_P2_11_EINT1
	lpc22xx_set_periph(LPC22XX_PIN_P2_11, 1, 0);	/* EINT1 */
#endif
	platform_device_register(&lpc22xx_eth_device);
}
#endif

/* --------------------------------------------------------------------
 *  USBs (Chapter 13, Chapter 14 and Chapter 15)
 * -------------------------------------------------------------------- */
#if defined(CONFIG_LPC22XX_USB1) || defined(CONFIG_LPC22XX_USB2)
static struct resource lpc22xx_usb_resource[] = {
	{
		.name	= "lpc22xx-usb",
		.start	= APH_USB_BASE,
		.end	= APH_USB_BASE + APH_USB_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "lpc22xx-usb-ram",
		.start	= USB_RAM_BASE,
		.end	= USB_RAM_BASE + USB_RAM_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device lpc22xx_usb_device = {
	.name		= "lpc22xx-usb",
	.id		= PLAT8250_DEV_PLATFORM,
	.num_resources	= ARRAY_SIZE(lpc22xx_usb_resource),
	.resource	= lpc22xx_usb_resource,
};

static void __init lpc22xx_add_usb_device(void)
{

	platform_device_register(&lpc22xx_usb_device);
}
#endif

/* --------------------------------------------------------------------
 *  CANs (Chapter 12)
 * -------------------------------------------------------------------- */
#if defined(CONFIG_LPC22XX_CAN1) || defined(CONFIG_LPC22XX_CAN2)
static struct resource lpc22xx_can_resource[] = {
	{
		.name	= "lpc22xx-can-b1",
		.start	= APB_CAN_B1_BASE,
		.end	= APB_CAN_B1_BASE + APB_CAN_B1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "lpc22xx-can-b2",
		.start	= APB_CAN_B2_BASE,
		.end	= APB_CAN_B2_BASE + APB_CAN_B2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "lpc22xx-can-b3",
		.start	= APB_CAN_B3_BASE,
		.end	= APB_CAN_B3_BASE + APB_CAN_B3_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#if defined(CONFIG_LPC22XX_CAN1)
	{
		.name	= "lpc22xx-can1",
		.start	= APB_CAN1_BASE,
		.end	= APB_CAN1_BASE + APB_CAN1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
#if defined(CONFIG_LPC22XX_CAN2)
	{
		.name	= "lpc22xx-can2",
		.start	= APB_CAN2_BASE,
		.end	= APB_CAN2_BASE + APB_CAN2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
};

static struct platform_device lpc22xx_can_device = {
	.name		= "lpc22xx-can",
	.id		= PLAT8250_DEV_PLATFORM,
	.num_resources	= ARRAY_SIZE(lpc22xx_can_resource),
	.resource	= lpc22xx_can_resource,
};

static void __init lpc22xx_add_can_device(void)
{

	platform_device_register(&lpc22xx_can_device);
}
#endif

/* --------------------------------------------------------------------
 *  PWMs (Chapter 25)
 * -------------------------------------------------------------------- */
#if defined(CONFIG_LPC22XX_PWM0) || defined(CONFIG_LPC22XX_PWM1)
static struct resource lpc22xx_pwm_resource[] = {
#if defined(CONFIG_LPC22XX_PWM0)
	{
		.name	= "lpc22xx-pwm0",
		.start	= APB_PWM0_BASE,
		.end	= APB_PWM0_BASE + APB_PWM0_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
#if defined(CONFIG_LPC22XX_PWM1)
	{
		.name	= "lpc22xx-pwm1",
		.start	= APB_PWM1_BASE,
		.end	= APB_PWM1_BASE + APB_PWM1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
};

static struct platform_device lpc22xx_pwm_device = {
	.name		= "lpc22xx-pwm",
	.id		= PLAT8250_DEV_PLATFORM,
	.num_resources	= ARRAY_SIZE(lpc22xx_pwm_resource),
	.resource	= lpc22xx_pwm_resource,
};

static void __init lpc22xx_add_pwm_device(void)
{
	platform_device_register(&lpc22xx_pwm_device);
}
#endif

/* --------------------------------------------------------------------
 *  ADC (Chapter 26)
 * -------------------------------------------------------------------- */
#if defined(CONFIG_LPC22XX_ADC)
static struct resource lpc22xx_adc_resource = {
	.name	= "lpc22xx-adc",
	.start	= APB_ADC_BASE,
	.end	= APB_ADC_BASE + APB_ADC_SIZE - 1,
	.flags	= IORESOURCE_MEM,
};
static struct platform_device lpc22xx_adc_device = {
	.name		= "lpc22xx-adc",
	.id		= -1,
	.num_resources	= 1,
	.resource	= &lpc22xx_adc_resource,
};

static void __init lpc22xx_add_adc_device(void)
{
	platform_device_register(&lpc22xx_adc_device);
}
#endif

/* --------------------------------------------------------------------
 *  DAC (Chapter 27)
 * -------------------------------------------------------------------- */
#if defined(CONFIG_LPC22XX_DAC)
static struct resource lpc22xx_dac_resource = {
	.name	= "lpc22xx-dac",
	.start	= APB_DAC_BASE,
	.end	= APB_DAC_BASE + APB_DAC_SIZE - 1,
	.flags	= IORESOURCE_MEM,
};
static struct platform_device lpc22xx_dac_device = {
	.name		= "lpc22xx-dac",
	.id		= -1,
	.num_resources	= 1,
	.resource	= &lpc22xx_dac_resource,
};

static void __init lpc22xx_add_dac_device(void)
{
	platform_device_register(&lpc22xx_dac_device);
}
#endif

/* --------------------------------------------------------------------
 *  WDT (Chapter 24)
 * -------------------------------------------------------------------- */
#if defined(CONFIG_LPC22XX_WDT)
static struct resource lpc22xx_wdt_resource = {
	.name	= "lpc22xx-wdt",
	.start	= APB_WDT_BASE,
	.end	= APB_WDT_BASE + APB_WDT_SIZE - 1,
	.flags	= IORESOURCE_MEM,
};
static struct platform_device lpc22xx_wdt_device = {
	.name		= "lpc22xx-wdt",
	.id		= -1,
	.num_resources	= 1,
	.resource	= &lpc22xx_wdt_resource,
};

static void __init lpc22xx_add_wdt_device(void)
{
	platform_device_register(&lpc22xx_wdt_device);
}
#endif

/* -------------------------------------------------------------------- */

/*
 * These devices are always present and don't need any board-specific
 * setup.
 */
static int __init lpc22xx_add_standard_devices(void)
{
	lpc22xx_add_global_device();
	lpc22xx_add_gpio_device();
#ifdef CONFIG_MTD_LPC22XX_FLASH
	lpc22xx_add_flash_device();
#endif
	lpc22xx_add_timer_device();
#if defined(CONFIG_LPC22XX_UART0) || defined(CONFIG_LPC22XX_UART1) || defined(CONFIG_LPC22XX_UART2) || defined(CONFIG_LPC22XX_UART3)
	lpc22xx_add_uart_device();
#endif
#if defined(CONFIG_LPC22XX_RTC)
	lpc22xx_add_rtc_device();
#endif
#if defined(CONFIG_LPC22XX_NVRAM)
	lpc22xx_add_bram_device();
#endif
#if defined(CONFIG_LPC22XX_I2C0)
	lpc22xx_add_i2c0_device();
#endif
#if defined(CONFIG_LPC22XX_I2C1)
	lpc22xx_add_i2c1_device();
#endif
#if defined(CONFIG_LPC22XX_I2C2)
	lpc22xx_add_i2c2_device();
#endif
#if defined(CONFIG_LPC22XX_SPI)
	lpc22xx_add_spi_device();
#endif
#if defined(CONFIG_LPC22XX_SSP0) || defined(CONFIG_LPC22XX_SSP1)
	lpc22xx_add_ssp_device();
#endif
#if defined(CONFIG_LPC22XX_I2S)
	lpc22xx_add_i2s_device();
#endif
#if defined(CONFIG_LPC22XX_MMC)
	lpc22xx_add_mmc_device();
#endif
#if defined(CONFIG_LPC22XX_ETH)
	lpc22xx_add_eth_device();
#endif
#if defined(CONFIG_LPC22XX_USB1) || defined(CONFIG_LPC22XX_USB2)
	lpc22xx_add_usb_device();
#endif
#if defined(CONFIG_LPC22XX_CAN1) || defined(CONFIG_LPC22XX_CAN2)
	lpc22xx_add_can_device();
#endif
#if defined(CONFIG_LPC22XX_PWM0) || defined(CONFIG_LPC22XX_PWM1)
	lpc22xx_add_pwm_device();
#endif
#if defined(CONFIG_LPC22XX_ADC)
	lpc22xx_add_adc_device();
#endif
#if defined(CONFIG_LPC22XX_DAC)
	lpc22xx_add_dac_device();
#endif
#if defined(CONFIG_LPC22XX_WDT)
	lpc22xx_add_wdt_device();
#endif
	return 0;
}

arch_initcall(lpc22xx_add_standard_devices);
