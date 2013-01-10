/*
 * Ethernet driver for the NXP LPC22XX
 *
 *  Copyright (C) 2007 Siemens Building Technologies (mailto:philippe.goetz@siemens.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#ifndef __LPC22XX_ETH_H
#define __LPC22XX_ETH_H

#include <asm/arch/lpc22xx.h>

/* APH=0 - Ethernet */
/* MAC registers */
#define REG_ETH_MAC1                (*((volatile unsigned long *) (APH_ETH_BASE+0x0000)))
#define REG_ETH_MAC2                (*((volatile unsigned long *) (APH_ETH_BASE+0x0004)))
#define REG_ETH_IPGT                (*((volatile unsigned long *) (APH_ETH_BASE+0x0008)))
#define REG_ETH_IPGR                (*((volatile unsigned long *) (APH_ETH_BASE+0x000C)))
#define REG_ETH_CLRT                (*((volatile unsigned long *) (APH_ETH_BASE+0x0010)))
#define REG_ETH_MAXF                (*((volatile unsigned long *) (APH_ETH_BASE+0x0014)))
#define REG_ETH_SUPP                (*((volatile unsigned long *) (APH_ETH_BASE+0x0018)))
#define REG_ETH_TEST                (*((volatile unsigned long *) (APH_ETH_BASE+0x001C)))
#define REG_ETH_MCFG                (*((volatile unsigned long *) (APH_ETH_BASE+0x0020)))
#define REG_ETH_MCMD                (*((volatile unsigned long *) (APH_ETH_BASE+0x0024)))
#define REG_ETH_MADR                (*((volatile unsigned long *) (APH_ETH_BASE+0x0028)))
#define REG_ETH_MWTD                (*((volatile unsigned long *) (APH_ETH_BASE+0x002C)))
#define REG_ETH_MRDD                (*((volatile unsigned long *) (APH_ETH_BASE+0x0030)))
#define REG_ETH_MIND                (*((volatile unsigned long *) (APH_ETH_BASE+0x0034)))
#define REG_ETH_SA0                 (*((volatile unsigned long *) (APH_ETH_BASE+0x0040)))
#define REG_ETH_SA1                 (*((volatile unsigned long *) (APH_ETH_BASE+0x0044)))
#define REG_ETH_SA2                 (*((volatile unsigned long *) (APH_ETH_BASE+0x0048)))
/* Control registers */
#define REG_ETH_COMMAND             (*((volatile unsigned long *) (APH_ETH_BASE+0x0100)))
#define REG_ETH_STATUS              (*((volatile unsigned long *) (APH_ETH_BASE+0x0104)))
#define REG_ETH_RXDESCRIPTOR        (*((volatile unsigned long *) (APH_ETH_BASE+0x0108)))
#define REG_ETH_RXSTATUS            (*((volatile unsigned long *) (APH_ETH_BASE+0x010C)))
#define REG_ETH_RXDESCRIPTORNUMBER  (*((volatile unsigned long *) (APH_ETH_BASE+0x0110)))
#define REG_ETH_RXPRODUCEINDEX      (*((volatile unsigned long *) (APH_ETH_BASE+0x0114)))
#define REG_ETH_RXCONSUMEINDEX      (*((volatile unsigned long *) (APH_ETH_BASE+0x0118)))
#define REG_ETH_TXDESCRIPTOR        (*((volatile unsigned long *) (APH_ETH_BASE+0x011C)))
#define REG_ETH_TXSTATUS            (*((volatile unsigned long *) (APH_ETH_BASE+0x0120)))
#define REG_ETH_TXDESCRIPTORNUMBER  (*((volatile unsigned long *) (APH_ETH_BASE+0x0124)))
#define REG_ETH_TXPRODUCEINDEX      (*((volatile unsigned long *) (APH_ETH_BASE+0x0128)))
#define REG_ETH_TXCONSUMEINDEX      (*((volatile unsigned long *) (APH_ETH_BASE+0x012C)))
#define REG_ETH_TSV0                (*((volatile unsigned long *) (APH_ETH_BASE+0x0158)))
#define REG_ETH_TSV1                (*((volatile unsigned long *) (APH_ETH_BASE+0x015C)))
#define REG_ETH_RSV                 (*((volatile unsigned long *) (APH_ETH_BASE+0x0160)))
#define REG_ETH_FLOWCONTROLCOUNTER  (*((volatile unsigned long *) (APH_ETH_BASE+0x0170)))
#define REG_ETH_FLOWCONTROLSTATUS   (*((volatile unsigned long *) (APH_ETH_BASE+0x0174)))
/* Rx filter registers */
#define REG_ETH_RXFILTERCTRL        (*((volatile unsigned long *) (APH_ETH_BASE+0x0200)))
#define REG_ETH_RXFILTERWOLSTATUS   (*((volatile unsigned long *) (APH_ETH_BASE+0x0204)))
#define REG_ETH_RXFILTERWOLCLEAR    (*((volatile unsigned long *) (APH_ETH_BASE+0x0208)))
#define REG_ETH_HASHFILTERL         (*((volatile unsigned long *) (APH_ETH_BASE+0x0210)))
#define REG_ETH_HASHFILTERH         (*((volatile unsigned long *) (APH_ETH_BASE+0x0214)))
/* Module control registers */
#define REG_ETH_INTSTATUS           (*((volatile unsigned long *) (APH_ETH_BASE+0x0FE0)))
#define REG_ETH_INTENABLE           (*((volatile unsigned long *) (APH_ETH_BASE+0x0FE4)))
#define REG_ETH_INTCLEAR            (*((volatile unsigned long *) (APH_ETH_BASE+0x0FE8)))
#define REG_ETH_INTSET              (*((volatile unsigned long *) (APH_ETH_BASE+0x0FEC)))
#define REG_ETH_POWERDOWN           (*((volatile unsigned long *) (APH_ETH_BASE+0x0FF4)))
#define REG_ETH_MODULEID            (*((volatile unsigned long *) (APH_ETH_BASE+0x0FFC)))

/* EMAC interrupt controller related definition */
#define KREG_ETH_INT_RXOVERRUN  (1 << 0)
#define KREG_ETH_INT_RXERROR    (1 << 1) 
#define KREG_ETH_INT_RXFINISHED (1 << 2)
#define KREG_ETH_INT_RXDONE     (1 << 3)
#define KREG_ETH_INT_TXUNDERRUN (1 << 4)
#define KREG_ETH_INT_TXERROR    (1 << 5)
#define KREG_ETH_INT_TXFINISHED (1 << 6)
#define KREG_ETH_INT_TXDONE     (1 << 7)
#define KREG_ETH_INT_SOFTINT    (1 << 12)
#define KREG_ETH_INT_WOL        (1 << 13) 

#define KREG_ETH_MODULEID_NXP   ((0x3902L << 16) | 0x2000L)
#define KREG_ETH_MODULEID_MASK	0xfffff000



/* Receive descriptor control word [Table 196] */
#define KREG_ETH_RXCONTROL_INT        0x80000000
#define KREG_ETH_RXCONTROL_SIZE_MASK  0x000007FF

/* Receive status information word [Table 199] */
#define KREG_ETH_RXSTATUS_ERR        0x80000000
#define KREG_ETH_RXSTATUS_LAST       0x40000000
#define KREG_ETH_RXSTATUS_NODESC     0x20000000
#define KREG_ETH_RXSTATUS_OVERRUN    0x10000000
#define KREG_ETH_RXSTATUS_ALGNERR    0x08000000
#define KREG_ETH_RXSTATUS_RNGERR     0x04000000
#define KREG_ETH_RXSTATUS_LENERR     0x02000000
#define KREG_ETH_RXSTATUS_SYMERR     0x01000000
#define KREG_ETH_RXSTATUS_CRCERR     0x00800000
#define KREG_ETH_RXSTATUS_BCAST      0x00400000
#define KREG_ETH_RXSTATUS_MCAST      0x00200000
#define KREG_ETH_RXSTATUS_FAILFLT    0x00100000
#define KREG_ETH_RXSTATUS_VLAN       0x00080000
#define KREG_ETH_RXSTATUS_CTLFRAM    0x00040000
#define KREG_ETH_RXSTATUS_SIZE_MASK  0x000007FF

/* Transmit descriptor control word [Table 201] */
#define KREG_ETH_TXCONTROL_INT       0x80000000
#define KREG_ETH_TXCONTROL_LAST      0x40000000
#define KREG_ETH_TXCONTROL_CRC       0x20000000
#define KREG_ETH_TXCONTROL_PAD       0x10000000
#define KREG_ETH_TXCONTROL_HUGE      0x08000000
#define KREG_ETH_TXCONTROL_OVERRIDE  0x04000000
#define KREG_ETH_TXCONTROL_SIZE_MASK 0x000007FF

/* Transmit status information word [Table 203] */
#define KREG_ETH_TXSTATUS_ERR        0x80000000
#define KREG_ETH_TXSTATUS_NODESC     0x40000000
#define KREG_ETH_TXSTATUS_UNDERRUN   0x20000000
#define KREG_ETH_TXSTATUS_LCOL       0x10000000
#define KREG_ETH_TXSTATUS_ECOL       0x08000000
#define KREG_ETH_TXSTATUS_EDEFER     0x04000000
#define KREG_ETH_TXSTATUS_DEFER      0x02000000
#define KREG_ETH_TXSTATUS_COLCNT_MASK 0x01E00000

/* Micrel_100001 PHY */
#define MII_MICREL_ID		0x00221400
#define MII_MICREL21_ID		(MII_MICREL_ID|0x210)
#define MII_MICREL_KSZ8001_ID	(MII_MICREL21_ID|0xA)
#define MII_MICREL_KSZ8721_ID	(MII_MICREL21_ID|0x19)

struct lpc22xx_private
{
	struct net_device_stats stats;
	struct mii_if_info mii;			/* ethtool support */
	struct lpc22xx_eth_data board_data;	/* board-specific configuration */
	/* PHY */
	unsigned short phy_address;		/* 5-bit MDI address of PHY (0..31) */
	unsigned long phy_type;			/* type of PHY (PHY_ID) */
	spinlock_t lock;			/* lock for MDI interface */
	struct timer_list check_timer;		/* Poll link status */
	/* Transmit */
	struct sk_buff *skb;			/* holds skb until xmit interrupt completes */
	int skb_length;				/* saved skb length for pci_unmap_single */
};

#endif
