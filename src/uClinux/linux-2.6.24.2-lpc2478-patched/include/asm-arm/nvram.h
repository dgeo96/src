/*
 * NVRAM definitions and access functions.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#ifndef _ASM_ARM_NVRAM_H
#define _ASM_Arm_NVRAM_H

#include <asm/arch/nvram.h>

#ifdef __KERNEL__
/* Normal access to NVRAM */
extern unsigned char nvram_read_byte(int i);
extern void nvram_write_byte(unsigned char c, int i);
/* Synchronize NVRAM */
extern void	nvram_sync(void);
#endif

/*
 * /dev/nvram ioctls
 *
 * Note that PMAC_NVRAM_GET_OFFSET is still supported, but is
 * definitely obsolete. Do not use it if you can avoid it
 */

#define OBSOLETE_PMAC_NVRAM_GET_OFFSET \
				_IOWR('p', 0x40, int)

#define IOC_NVRAM_GET_OFFSET	_IOWR('p', 0x42, int)	/* Get NVRAM partition offset */
#define IOC_NVRAM_SYNC		_IO('p', 0x43)		/* Sync NVRAM image */

#endif /* _ASM_ARM_NVRAM_H */
