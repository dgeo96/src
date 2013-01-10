/*
 *	Linux Internal Flash driver for NXP LPC22XX
 *
 *	Copyright (C) 2007 Siemens Building Technologies
 *	                   mailto:philippe.goetz@siemens.com
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 * Reference:
 * [1] LPC2468 User Manual Chapter 29
 *
 * Programmer Guideline:
 * - platdorm_device.dev.platform_data should supply the mtd_partitions 
 *   applicable to the device.
 * 
 * Limitations:
 * - This driver can only manage ONE related platform device (see static).
 * - This driver only works in conjunction with a platform device structure.
 */
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/irqflags.h>
#include <asm/arch/lpc22xx.h>

//#define MTD_LPC22XX_FLASH_DEBUG 1

static char MODULE_NAME[] = "lpc22xx-flash";


#define LPC22XX_FLASH_WRITING_BLOCK_BOUNDARY 256  /* DON'T CHANGE THIS VALUE */
#define LPC22XX_FLASH_WRITING_BLOCK_SIZE 256      /* 256|512|1024|4096 */

/* IAP commands use 32 bytes at the top of CPU internal sram, we
   use 512 bytes below that */
#define IAP_COPY_BUFFER_LOCATION (0x4000ffe0-LPC22XX_FLASH_WRITING_BLOCK_SIZE)

#define IAP_LOCATION 0x7ffffff1
#define IAP_CMD_PREPARE 50
#define IAP_CMD_COPY 51
#define IAP_CMD_ERASE 52
#define IAP_CMD_CHECK 53
#define IAP_CMD_ID 54
#define IAP_CMD_VERSION 55
#define IAP_CMD_COMPARE 56
static unsigned long iap_location = IAP_LOCATION;
static unsigned long iap_command[5];
static unsigned long iap_result[2];

static void iap_entry(unsigned long * command, unsigned long * result) {
	register long _r0 asm("r0")=(long)(command);
	register long _r1 asm("r1")=(long)(result);	
	register long _r2 asm("r2")=iap_location;	
	asm volatile(	"mov lr, pc\n\t"
			"bx r2\n\t"
			:
			: "r"(_r0), "r"(_r1), "r"(_r2)
			: "memory");
}

static int iap_prepare(unsigned long start_sector, unsigned long end_sector)
{
	iap_command[0] = IAP_CMD_PREPARE;
	iap_command[1] = start_sector;
	iap_command[2] = end_sector;
	iap_entry(iap_command, iap_result);
	return iap_result[0];
}

static int iap_write(unsigned long dst, const u_char *src, unsigned long size)
{
	unsigned long flags;

	memcpy((void*)IAP_COPY_BUFFER_LOCATION, src, size);

	iap_command[0] = IAP_CMD_COPY;
	iap_command[1] = dst;
	iap_command[2] = IAP_COPY_BUFFER_LOCATION;
	iap_command[3] = size;
	iap_command[4] = LPC22xx_Fcclk >> 10;

	raw_local_irq_save(flags);
	iap_entry(iap_command, iap_result);
	raw_local_irq_restore(flags);

	return iap_result[0];
}

static int iap_erase(unsigned long start_sector, unsigned long end_sector)
{
	unsigned long flags;

	iap_command[0] = IAP_CMD_ERASE;
	iap_command[1] = start_sector;
	iap_command[2] = end_sector;
	iap_command[3] = LPC22xx_Fcclk >> 10;

	raw_local_irq_save(flags);
	iap_entry(iap_command, iap_result);
	raw_local_irq_restore(flags);

	return iap_result[0];
}

static struct mutex mtd_lpc22xx_flash_lock;

static int mtd_lpc22xx_flash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int result;
	unsigned long start_sector, end_sector;
	struct mtd_erase_region_info *regions = mtd->eraseregions;
	int region;

	if (instr->addr > mtd->size) {
		return -EINVAL;
	}

	if ((instr->len + instr->addr) > mtd->size) {
		return -EINVAL;
	}

	start_sector = 0;
	region = 0;
	/* Skip all erase regions which are ended before the start of
	 * the requested erase. Actually, to save on the calculations,
	 * we skip to the first erase region which starts after the
	 * start of the requested erase, and then go back one.
	 */
	while((region < mtd->numeraseregions) && (instr->addr >= regions[region].offset)) {
		start_sector += regions[region].numblocks;
		region++;
	}
	region--;
	start_sector -= regions[region].numblocks;
	/* OK, now i is pointing at the erase region in which this
	 * erase request starts. Check the start of the requested
	 * erase range is aligned with the erase size which is in
	 * effect here.
	 */
	if(instr->addr & (regions[region].erasesize-1)) {
		return -EINVAL;
	}
	end_sector = start_sector;
	start_sector += (instr->addr - regions[region].offset) / regions[region].erasesize;

	/* Next, check that the end of the requested erase is aligned
	 * with the erase region at that address.
	 */
	while ((region < mtd->numeraseregions) && ((instr->addr + instr->len - 1) >= regions[region].offset)) {
		end_sector += regions[region].numblocks;
		region++;
	}
	/* As before, drop back one to point at the region in which
	 * the address actually falls.
	 */
	region--;
	end_sector -= regions[region].numblocks;
	if ((instr->addr + instr->len) & (regions[region].erasesize-1)) {
		return -EINVAL;
	}
	end_sector += ((instr->addr + instr->len - 1) - regions[region].offset) / regions[region].erasesize;

	mutex_lock(&mtd_lpc22xx_flash_lock);

	instr->state = MTD_ERASING;

	result = iap_prepare(start_sector, end_sector);
	if(result!=0) {
		instr->state = MTD_ERASE_FAILED;
		goto callback;
	}
	result = iap_erase(start_sector, end_sector);
	if(result!=0) {
		instr->state = MTD_ERASE_FAILED;
		goto callback;
	}

	instr->state = MTD_ERASE_DONE;

callback:
	mtd_erase_callback(instr);

	mutex_unlock(&mtd_lpc22xx_flash_lock);

	return 0;
}

static int mtd_lpc22xx_flash_read(struct mtd_info *mtd, loff_t from, size_t len, size_t *retlen, u_char *buf)
{
	*retlen = 0;
	if ((from + len) > mtd->size) {
		return -EINVAL;
	}

	mutex_lock(&mtd_lpc22xx_flash_lock);

	if(from<0x40) {
		unsigned long memmap;
		memmap = MEMMAP;
		// If the Interrupt Vectors are not mapped in the Flash, then
		// temporarirely remap the Flash in the memory to read it
		// (disable the interrupts for this operation!!).
		if((memmap&3)!=MEMMAP_USER_FLASH) {
			unsigned long flags;
			raw_local_irq_save(flags);
			MEMMAP = MEMMAP_USER_FLASH;
			memcpy(buf, (void *)from, len);
			MEMMAP = memmap;
			raw_local_irq_restore(flags);
			goto exit;
		}
	}

	memcpy(buf, (void *)from, len);
exit:
	*retlen = len;
	mutex_unlock(&mtd_lpc22xx_flash_lock);

	return 0;
}

static int mtd_lpc22xx_flash_write(struct mtd_info *mtd, loff_t to, size_t len, size_t *retlen, const u_char *buf)
{
	int result;
	unsigned long start_sector, end_sector, addr, offset;
	struct mtd_erase_region_info *regions = mtd->eraseregions;
	int region;

	*retlen = 0;
	addr = to;
	if (addr > mtd->size) {
		return -EINVAL;
	}

	if ((len + addr) > mtd->size) {
		return -EINVAL;
	}

	start_sector = 0;
	region = 0;
	/* Skip all erase regions which are ended before the start of
	 * the requested write. Actually, to save on the calculations,
	 * we skip to the first erase region which starts after the
	 * start of the requested erase, and then go back one.
	 */
	while((region < mtd->numeraseregions) && (addr >= regions[region].offset)) {
		start_sector += regions[region].numblocks;
		region++;
	}
	region--;
	start_sector -= regions[region].numblocks;
	/* OK, now i is pointing at the erase region in which this
	 * write request starts.
	 */
	end_sector = start_sector;
	start_sector += (addr - regions[region].offset) / regions[region].erasesize;

	/* Next, check that the end of the requested erase is aligned
	 * with the erase region at that address.
	 */
	while ((region < mtd->numeraseregions) && ((addr + len - 1) >= regions[region].offset)) {
		end_sector += regions[region].numblocks;
		region++;
	}
	/* As before, drop back one to point at the region in which
	 * the address actually falls.
	 */
	region--;
	end_sector -= regions[region].numblocks;

	end_sector += ((addr + len - 1) - regions[region].offset) / regions[region].erasesize;

	mutex_lock(&mtd_lpc22xx_flash_lock);

	for(offset = 0; offset < len; offset += mtd->writesize) {
		result = iap_prepare(start_sector, end_sector);
		if(result!=0) {
			mutex_unlock(&mtd_lpc22xx_flash_lock);
			return -EINVAL;		}
		result = iap_write(addr+offset, buf+offset, mtd->writesize);
		if(result!=0) {
			mutex_unlock(&mtd_lpc22xx_flash_lock);
			return -EINVAL;
		}
	}
	*retlen = len;

	mutex_unlock(&mtd_lpc22xx_flash_lock);

	return 0;
}

/*****************************************************************************/

/* The following defines describe the physical layout of the Flash */
#define FLASH_BLOCK_PARAM_SIZE 4096
#define FLASH_BLOCK_MAIN_SIZE 32768
#define FLASH_BLOCK_PARAM_BOTTOM 8
#define FLASH_BLOCK_MAIN 14
#define FLASH_BLOCK_PARAM_TOP 8

/* The following MTD structure describes the physical layout of the Flash */
static struct mtd_erase_region_info erase_regions[] = {
        /* bottom blocks */
        {
                .offset         = 0,
                .erasesize      = FLASH_BLOCK_PARAM_SIZE,
                .numblocks      = FLASH_BLOCK_PARAM_BOTTOM,
        },
        /* main blocks */
        {
                .offset         = FLASH_BLOCK_PARAM_SIZE * FLASH_BLOCK_PARAM_BOTTOM,
                .erasesize      = FLASH_BLOCK_MAIN_SIZE,
                .numblocks      = FLASH_BLOCK_MAIN,
        },
        /* top blocks */
        {
                .offset         = FLASH_BLOCK_PARAM_SIZE * FLASH_BLOCK_PARAM_BOTTOM +
                                  FLASH_BLOCK_MAIN_SIZE * FLASH_BLOCK_MAIN,
                .erasesize      = FLASH_BLOCK_PARAM_SIZE,
                .numblocks      = FLASH_BLOCK_PARAM_TOP,
        },
};

static int mtd_lpc22xx_flash_remove(struct platform_device *dev)
{
//	struct mtd_info *mtd = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	return 0;
}

static int mtd_lpc22xx_flash_probe(struct platform_device *dev)
{
	static struct mtd_info mtd;
	struct mtd_partition *partition_info = dev->dev.platform_data;
	int partition_nr;
	int err = -1;

	memzero(&mtd, sizeof(struct mtd_info));

	platform_set_drvdata(dev, &mtd);

	mtd.name = MODULE_NAME;
	mtd.type = MTD_NORFLASH;
	mtd.writesize = LPC22XX_FLASH_WRITING_BLOCK_SIZE;
	// If the Interrupt Vectors are mapped in the Flash, make the Flash
	// read-only to avoid unpredictable behaviour on writing.
	if((MEMMAP&3)!=MEMMAP_USER_FLASH) mtd.flags = MTD_WRITEABLE;
	mtd.size = FLASH_BLOCK_PARAM_SIZE * (FLASH_BLOCK_PARAM_BOTTOM + FLASH_BLOCK_PARAM_TOP) +
	                 FLASH_BLOCK_MAIN_SIZE * FLASH_BLOCK_MAIN;
	mtd.erasesize = FLASH_BLOCK_PARAM_SIZE;
	mtd.numeraseregions = ARRAY_SIZE(erase_regions);
	mtd.eraseregions = erase_regions;
	mtd.erase = mtd_lpc22xx_flash_erase;
	mtd.read = mtd_lpc22xx_flash_read;
	mtd.write = mtd_lpc22xx_flash_write;
	mtd.owner = THIS_MODULE;

	mutex_init(&mtd_lpc22xx_flash_lock);

#ifdef MTD_LPC22XX_FLASH_DEBUG
   printk (KERN_DEBUG
		   "mtd.name = %s\n"
		   "mtd.size = 0x%.8x (%uM)\n"
		   "mtd.erasesize = 0x%.8x (%uK)\n"
		   "mtd.numeraseregions = %d\n",
		   mtd.name,
		   mtd.size,mtd.size / (1024*1024),
		   mtd.erasesize,mtd.erasesize / 1024,
		   mtd.numeraseregions);

   if (mtd.numeraseregions)
	 for (partition_nr = 0; partition_nr < mtd.numeraseregions; partition_nr++)
	   printk (KERN_DEBUG
			   "\n\n"
			   "mtd.eraseregions[%d].offset = 0x%.8x\n"
			   "mtd.eraseregions[%d].erasesize = 0x%.8x (%uK)\n"
			   "mtd.eraseregions[%d].numblocks = %d\n",
			   partition_nr,mtd.eraseregions[partition_nr].offset,
			   partition_nr,mtd.eraseregions[partition_nr].erasesize,mtd.eraseregions[partition_nr].erasesize / 1024,
			   partition_nr,mtd.eraseregions[partition_nr].numblocks);

   if(partition_info) {
         for (partition_nr = 0; partition_info[partition_nr].name; partition_nr++)
	       printk (KERN_DEBUG
			 "\n\n"
			 "partition_info[%d].name = %s\n"
			 "partition_info[%d].offset = 0x%.8x\n"
			 "partition_info[%d].size = 0x%.8x (%uK)\n",
			 partition_nr,partition_info[partition_nr].name,
			 partition_nr,partition_info[partition_nr].offset,
			 partition_nr,partition_info[partition_nr].size,partition_info[partition_nr].size / 1024);
   }
#endif

	if(partition_info) {
		for(partition_nr = 0; partition_info[partition_nr].name ; partition_nr++);
		err = add_mtd_partitions(&mtd, partition_info, partition_nr);
		if(err) {
			printk(KERN_ERR "mtd-lpc22xx-flash: Could not add partitions\n");
			goto Error;
		}
	}
	else {  
		err = add_mtd_device (&mtd);
		if(err) {
			printk(KERN_ERR "mtd-lpc22xx-flash: Could not add device\n");
			goto Error;
		}
	}

	printk(KERN_INFO "lpc22xx-flash: MTD driver for NXP LPC22XX Internal Flash.\n");

	return 0;

Error:
	mtd_lpc22xx_flash_remove(dev);
	return err;
}

static struct platform_driver mtd_lpc22xx_flash_driver = {
	.probe		= mtd_lpc22xx_flash_probe,
	.remove		= mtd_lpc22xx_flash_remove,
	.driver		= {
		.name	= MODULE_NAME,
		.owner  = THIS_MODULE,
	},
};

static int __init mtd_lpc22xx_flash_init(void)
{
	return platform_driver_register(&mtd_lpc22xx_flash_driver);
}

static void __exit mtd_lpc22xx_flash_exit(void)
{
	platform_driver_unregister(&mtd_lpc22xx_flash_driver);
}

module_init(mtd_lpc22xx_flash_init);
module_exit(mtd_lpc22xx_flash_exit);

MODULE_AUTHOR("Siemens Building Technologies, Philippe GOETZ <mailto:philippe.goetz@siemens.com>");
MODULE_DESCRIPTION("MTD driver for NXP LPC22XX Internal Flash");
MODULE_LICENSE("GPL");
