/*
 *  linux/arch/arm/mach-lpc22xx/lpc2468_ea_board.c
 *
 *  Author: Philippe GOETZ (philippe dot goetz at siemens dot com)
 *
 *  Copyright (C) 2007 Siemens Building Technologies
 *
 * This file is used to define the board related platform devices and
 * MTD partitions.
 *
 * The platform devices are:
 * - External NOR Flash
 * - External NAND Flash
 *
 * The MTD partitions are:
 * - Internal Flash with 4 partitions:
 *   - u-boot
 *   - file system
 *   - u-boot environment
 *   - flash boot
 * - NOR Flash with 1 or 2 partitions:
 *   - kernel in NOR
 *   - file system
 * - NAND Flash with 1 or 2 partitions:
 *   - kernel in NAND
 *   - file system (can be splitted in more partitions)
 *
 */

#include <linux/platform_device.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/setup.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>

#include <asm/arch/lpc22xx.h>

#ifdef CONFIG_MTD_NAND
/* The NAND is a K9F1G08U0A 128Mx8 located on CS1 [0x81000000-0x81FFFFFF]
 * NAND     CPU
 * ALE    = A19
 * CLE    = A20
 * !CE    = CS1
 * !RE    = OE
 * !WE    = WE
 * !WP    = 3V3
 * !R/B   = P2.12 (optional) + pull-up
*/ 

#define NAND_BASE (EXTERNAL_SMEM_BASE+(1*EXTERNAL_SBLOCK))
#define NAND_SIZE EXTERNAL_SBLOCK

static struct mtd_info * lpc2468_ea_nand_mtd = NULL;
static void __iomem *lpc2468_ea_nand_base;

#ifdef CONFIG_MTD_PARTITIONS
static struct mtd_partition lpc2468_ea_nand_partition_info[] = {
	{
		.name	= "Linux",
		.offset	= 0,
		.size	= 2 * SZ_1M,
	},
	{
		.name	= "ROMfs",
		.offset	= 2 * SZ_1M,
		.size	= 2 * SZ_1M,
	},
	{
		.name	= "User",
		.offset	= 4 * SZ_1M,
		.size	= 124 * SZ_1M,
	},
};
#endif

/*
 *	hardware specific access to control-lines
 *
 *	NCE IO_ADDR_W
 *	ALE IO_ADDR_W+(1<<19)
 *	CLE IO_ADDR_W+(1<<20)
 *
 */
static void lpc2468_ea_nand_hwcontrol(struct mtd_info *mtd, int cmd,
			       unsigned int ctrl)
{
	void __iomem *addr = lpc2468_ea_nand_base;
	if(ctrl & NAND_ALE)
		addr = addr+(1L<<19);
	else if(ctrl & NAND_CLE)
		addr = addr+(1L<<18);
	if (cmd != NAND_CMD_NONE)
		writeb(cmd, addr);
}

/*
 *	read device ready pin
 * int lpc2468_ea_nand_device_ready(struct mtd_info *mtd)
 */

static int __init lpc2468_ea_nand_init(void)
{
	struct nand_chip *this;
	int err = 0;

	/* Allocate memory for MTD device structure and private data */
	lpc2468_ea_nand_mtd = kmalloc(sizeof(struct mtd_info)+sizeof(struct nand_chip), GFP_KERNEL);
	if(!lpc2468_ea_nand_mtd) {
		printk("Unable to allocate NAND MTD device structure.\n");
		err = -EIO;
		goto out;
	}

	/* Map physical adress */
	lpc2468_ea_nand_base = ioremap(NAND_BASE, NAND_SIZE);
	if (!lpc2468_ea_nand_base) {
		printk("Ioremap NAND MTD chip fails.\n");
		err = -EIO;
		goto out_mtd;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *)(&lpc2468_ea_nand_mtd[1]);

	/* Initialize structures */
	memset(lpc2468_ea_nand_mtd, 0, sizeof(struct mtd_info));
	memset(this, 0, sizeof(struct nand_chip));

	/* Link the private data with the MTD structure */
	lpc2468_ea_nand_mtd->priv = this;
	lpc2468_ea_nand_mtd->owner = THIS_MODULE;

	/* Set address of NAND IO lines */
	this->IO_ADDR_R = lpc2468_ea_nand_base;
	this->IO_ADDR_W = lpc2468_ea_nand_base;
	this->cmd_ctrl = lpc2468_ea_nand_hwcontrol;
	/*
	this->dev_ready = lpc2468_ea_nand_device_ready;
	*/
	this->chip_delay = 25; /* 25 us command delay time */
	this->ecc.mode = NAND_ECC_SOFT;
	/*
	this->options = NAND_USE_FLASH_BBT;
	*/

	/* Scan to find existance of the device */
	if (nand_scan(lpc2468_ea_nand_mtd, 1)) {
		err = -ENXIO;
		goto out_ior;
	}
#ifdef CONFIG_MTD_PARTITIONS
	/* Register the partitions */
	add_mtd_partitions(lpc2468_ea_nand_mtd, lpc2468_ea_nand_partition_info,
					   ARRAY_SIZE(lpc2468_ea_nand_partition_info));
#endif
	goto out;
out_ior:
	iounmap(lpc2468_ea_nand_base);
out_mtd:
	kfree(lpc2468_ea_nand_mtd);
	lpc2468_ea_nand_mtd = NULL;

out:
	return err;
}

static void __exit lpc2468_ea_nand_cleanup(void)
{
	if(lpc2468_ea_nand_mtd != NULL) {
		/* Release resources, unregister device */
		nand_release(lpc2468_ea_nand_mtd);

		/* Unmap physical adress */
		iounmap(lpc2468_ea_nand_base);

		/* Free the MTD device structure */
		kfree(lpc2468_ea_nand_mtd);
	}
}
#endif /* CONFIG_MTD_NAND */

#ifdef CONFIG_MTD_LPC22XX_FLASH

#ifdef CONFIG_MTD_PARTITIONS
struct mtd_partition lpc22xx_flash_partition_info[] = {
	{
		.name	= "U-boot",
		.offset	= 0x00000000,
		.size	= 0x00030000, /* up to 192k */
	},
	{
		.name	= "User1",
		.offset	= 0x00030000,
		.size	= 0x00048000,
	},
	{
		.name	= "User2",
		.offset	= 0x00078000,
		.size	= 0x00004000,
	},
	{
		.name	= "U-boot params",
		.offset	= 0x0007C000,
		.size	= 0x00002000, /* 8k only 4k effectively used */
	},
	{
		.name	= "Flash Boot Loader",
		.offset	= 0x0007E000,
		.size	= 0x00002000, /* 8k read-only */
	},
	{
	}
};
#endif

#endif /* CONFIG_LPC22XX_FLASH */

static int __init lpc2468_ea_init(void)
{
#ifdef CONFIG_MTD_NAND
	lpc2468_ea_nand_init();
#endif /* CONFIG_MTD_NAND */
	return 0;
}

static void __exit lpc2468_ea_cleanup(void)
{
#ifdef CONFIG_MTD_NAND
	lpc2468_ea_nand_cleanup();
#endif /* CONFIG_MTD_NAND */
}

module_init(lpc2468_ea_init);
module_exit(lpc2468_ea_cleanup);

#ifdef CONFIG_MTD_LPC2468_EA_FLASH
#ifdef CONFIG_MTD_PARTITIONS
static struct mtd_partition lpc2468_ea_flash_partition_info[] = {
	{
		.name	= "Data1",
		.offset	= 0x00000000,
		.size	= 0x00200000,
	},
	{
		.name	= "Data2",
		.offset	= 0x00200000,
		.size	= 0x00200000,
	},
	{
	}
};
#endif

static struct resource lpc2468_ea_flash_resources[] = {
	{
		.name	= "lpc2468-ea-flash",
		.start	= EXTERNAL_SMEM_BASE,
		.end	= EXTERNAL_SMEM_BASE + EXTERNAL_SBLOCK - 1,
		.flags	= IORESOURCE_MEM,
	},
};
static struct platform_device lpc2468_ea_flash_device = {
	.name		= "lpc2468-ea-flash",
	.id		= -1,
#ifdef CONFIG_MTD_PARTITIONS
	.dev		= {
		.platform_data	= lpc2468_ea_flash_partition_info,
	},
#endif
	.num_resources	= ARRAY_SIZE(lpc2468_ea_flash_resources),
	.resource	= lpc2468_ea_flash_resources,
};

static int __init lpc2468_ea_add_flash_device(void)
{
	platform_device_register(&lpc2468_ea_flash_device);
	return 0;
}
#endif /* CONFIG_MTD_LPC2468_EA_FLASH */

static int __init lpc2468_ea_add_standard_devices(void)
{
#ifdef CONFIG_MTD_LPC2468_EA_FLASH
	lpc2468_ea_add_flash_device();
#endif /* CONFIG_MTD_LPC2468_EA_FLASH */
	return 0;
}

arch_initcall(lpc2468_ea_add_standard_devices);

extern void __init lpc22xx_init_irq(void);

void __init lpc2468_ea_init_machine(void)
{
}

#ifdef CONFIG_BOARD_LPC2468_EA_BOOTLOADER
static void __init lpc2468_ea_fixup(struct machine_desc *desc,
		struct tag *tags, char **cmdline, struct meminfo *mi)
{
	mi->nr_banks=1;
	mi->bank[0].start = CONFIG_DRAM_BASE;
	mi->bank[0].size = CONFIG_DRAM_SIZE;
	mi->bank[0].node = PHYS_TO_NID(mi->bank[0].start);
}
#endif

extern struct sys_timer lpc22xx_timer;

MACHINE_START(LPC22XX, "Embedded Artists LPC2468 OEM Board")
/*	MAINTAINER("Philippe Goetz <philippe.goetz@siemens.com>")    */
	.init_irq = lpc22xx_init_irq,
	.timer	= &lpc22xx_timer,
	.init_machine = lpc2468_ea_init_machine,
#ifdef CONFIG_BOARD_LPC2468_EA_BOOTLOADER
	.fixup = lpc2468_ea_fixup,
	.boot_params = CONFIG_BOARD_LPC2468_EA_BOOTARGS,
#endif
MACHINE_END

MODULE_LICENSE("GPL");
MODULE_AUTHOR("XXX");
MODULE_DESCRIPTION("Embedded Artists LPC2468 OEM Board description");
