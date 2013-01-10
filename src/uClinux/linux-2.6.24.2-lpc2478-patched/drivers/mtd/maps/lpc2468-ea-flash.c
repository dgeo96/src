/*
 *	Linux Embedded Artists LPC2468 OEM Board NOR Flash mapping driver
 *
 *	Copyright (C) 2007 Siemens Building Technologies
 *	                   mailto:philippe.goetz@siemens.com
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 *	This mapping will fix the design error which connects:
 *      - CPU_A22 to FLASH_A19 (eq. CPU_A20)
 */
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

static inline u16 flash_read16(void __iomem *addr)
{
	u16 d;
	// If A20 is set, then set also A22 so the flash seems linear.
	if(((unsigned long)addr) & (1L<<20)) {
		addr = (void __iomem *)(((unsigned long)addr) | (1L<<22));
	}
	d = __raw_readw(addr);
	return d;
}

static inline void flash_write16(u16 d, void __iomem *addr)
{
	// If A20 is set, then set also A22 so the flash seems linear.
	if(((unsigned long)addr) & (1L<<20)) {
		addr = (void __iomem *)(((unsigned long)addr) | (1L<<22));
	}
	__raw_writew(d, addr);
}

static map_word lpc2468_ea_read16(struct map_info *map, unsigned long ofs)
{
	map_word val;
	val.x[0] = flash_read16(map->virt + ofs);
	return val;
}

/*
 * Unaligned writes are ignored, causing the 8-bit
 * probe to fail and proceed to the 16-bit probe (which succeeds).
 */
static void lpc2468_ea_probe_write16(struct map_info *map, map_word d, unsigned long adr)
{
	if (!(adr & 1))
		flash_write16(d.x[0], map->virt + adr);
}

static void lpc2468_ea_write16(struct map_info *map, map_word d, unsigned long adr)
{
	flash_write16(d.x[0], map->virt + adr);
}

#define	BYTE0(h)	((h) & 0xFF)
#define	BYTE1(h)	(((h) >> 8) & 0xFF)

static void lpc2468_ea_copy_from(struct map_info *map, void *to,
			     unsigned long from, ssize_t len)
{
	u8 *dest = (u8 *) to;
	void __iomem *src = map->virt + from;

	if (len <= 0)
		return;

	if (from & 1) {
		*dest++ = BYTE1(flash_read16(src));
                src++;
		--len;
	}

	while (len >= 2) {
		u16 data = flash_read16(src);
		*dest++ = BYTE0(data);
		*dest++ = BYTE1(data);
		src += 2;
		len -= 2;
        }

	if (len > 0)
		*dest++ = BYTE0(flash_read16(src));
}

struct lpc2468_ea_flash_info {
	struct		mtd_info *mtd;
	struct		map_info map;
	struct		resource *res;
};

static int lpc2468_ea_flash_remove(struct platform_device *dev)
{
	struct lpc2468_ea_flash_info *info = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	if(!info)
		return 0;

	if (info->mtd) {
		del_mtd_partitions(info->mtd);
		map_destroy(info->mtd);
	}
	if (info->map.virt)
		iounmap(info->map.virt);

	if (info->res) {
		release_resource(info->res);
		kfree(info->res);
	}

	kfree(info);

	return 0;
}

static int lpc2468_ea_flash_probe(struct platform_device *dev)
{
	struct mtd_partition *partition_info = dev->dev.platform_data;
	int partition_nr;
	struct lpc2468_ea_flash_info *info;
	int err = -1;

	info = kmalloc(sizeof(struct lpc2468_ea_flash_info), GFP_KERNEL);
	if(!info) {
		printk(KERN_ERR "lpc2468-ea-flash: Could not allocate info structure\n");
		err = -ENOMEM;
		goto Error;
	}
	memzero(info, sizeof(struct lpc2468_ea_flash_info));

	platform_set_drvdata(dev, info);

	/*
	 * Tell the MTD layer we're not 1:1 mapped so that it does
	 * not attempt to do a direct access on us.
	 */
	info->map.phys = NO_XIP;
	info->map.size = dev->resource->end - dev->resource->start + 1;

	/*
	 * We only support 16-bit accesses for now. If and when
	 * any board use 8-bit access, we'll fixup the driver to
	 * handle that.
	 */
	info->map.bankwidth = 2;
	info->map.name = dev->dev.bus_id;
	info->map.read = lpc2468_ea_read16;
	info->map.copy_from = lpc2468_ea_copy_from;
	info->map.write = lpc2468_ea_probe_write16;
	/* info->map.copy_to = lpc2468_ea_copy_to; */
	info->res = request_mem_region(dev->resource->start,
			dev->resource->end - dev->resource->start + 1,
			"lpc2468-ea-flash");
	if (!info->res) {
		printk(KERN_ERR "lpc2468-ea-flash: Could not reserve memory region\n");
		err = -ENOMEM;
		goto Error;
	}

	info->map.virt = ioremap(dev->resource->start,
				 dev->resource->end - dev->resource->start + 1);
	if (!info->map.virt) {
		printk(KERN_ERR "lpc2468-ea-flash: Failed to ioremap region\n");
		err = -EIO;
		goto Error;
	}
	info->mtd = do_map_probe("jedec_probe", &info->map);
	if (!info->mtd) {
		printk(KERN_ERR "lpc2468-ea-flash: map_probe failed\n");
		err = -ENXIO;
		goto Error;
	}
	info->mtd->owner = THIS_MODULE;

	/* Use the fast version */
	info->map.write = lpc2468_ea_write16;

	if(partition_info) {
		for(partition_nr = 0; partition_info[partition_nr].name ; partition_nr++);
		err = add_mtd_partitions(info->mtd, partition_info, partition_nr);
		if(err) {
			printk(KERN_ERR "lpc2468-ea-flash: Could not add partitions\n");
			goto Error;
		}
	}

	return 0;

Error:
	lpc2468_ea_flash_remove(dev);
	return err;
}

static struct platform_driver lpc2468_ea_flash_driver = {
	.probe		= lpc2468_ea_flash_probe,
	.remove		= lpc2468_ea_flash_remove,
	.driver		= {
		.name	= "lpc2468-ea-flash",
		.owner  = THIS_MODULE,
	},
};

static int __init lpc2468_ea_flash_init(void)
{
	return platform_driver_register(&lpc2468_ea_flash_driver);
}

static void __exit lpc2468_ea_flash_exit(void)
{
	platform_driver_unregister(&lpc2468_ea_flash_driver);
}

module_init(lpc2468_ea_flash_init);
module_exit(lpc2468_ea_flash_exit);

MODULE_AUTHOR("Siemens Building Technologies, Philippe GOETZ <mailto:philippe.goetz@siemens.com>");
MODULE_DESCRIPTION("MTD mapping driver for Embedded Artists LPC2468 OEM Board NOR Flash");
MODULE_LICENSE("GPL");
