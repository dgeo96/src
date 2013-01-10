/*
 * include/asm-arm/arch-lpc22xx/board.h
 *
 *  Copyright (C) 2007 Siemens Building Technologies
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * These are data structures found in platform_device.dev.platform_data,
 * and describing board-specific data needed by drivers.  For example,
 * which pin is used for a given GPIO role.
 *
 * In 2.6, drivers should strongly avoid board-specific knowledge so
 * that supporting new boards normally won't require driver patches.
 * Most board-specific knowledge should be in arch/.../board-*.c files.
 */

#ifndef __ASM_ARCH_BOARD_H
#define __ASM_ARCH_BOARD_H


 /* Ethernet */
struct lpc22xx_eth_data {
};
extern void __init lpc22xx_add_device_eth(struct lpc22xx_eth_data *data);

#endif
