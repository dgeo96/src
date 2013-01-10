/****************************************************************************
 *  
 * Project: uClinux support for LPC-2478-STK
 *
 * Copyright: Ivan Vasilev, Olimex Ltd. All rights reserved.
 * 
 * File: $File lpc2478fb.h $
 * Description: Framebuffer include file
 * Developer: Ivan Vasilev, <ivan at l123.org>
 *
 * Last change: $Date: 2008-04-08 09:23:26 +0300 (вторник, 08 Април 2008) $
 * Revision: $Revision: 4 $
 * Id: $Id: lpc2478fb.h  4 2008-04-08 06:23:26Z Ivan $
 * Author: $Author: Ivan $
 *
 ****************************************************************************/

 
#define	LCD_BASE	0xffe10000
#define	LCD_CFG		(*(volatile unsigned long *)(0xe01fc1b8))	/* LCD configuration and clocking control */
#define	LCD_CFG_CLKDIV	0	/* bifield offset */

#define	LCD_TIMH	(*(volatile unsigned long *)(LCD_BASE + 0x000))
#define	LCD_TIMH_HBP	24
#define	LCD_TIMH_HFP	16
#define	LCD_TIMH_HSW	8
#define	LCD_TIMH_PPL	2

#define	LCD_TIMV	(*(volatile unsigned long *)(LCD_BASE + 0x004))
#define	LCD_TIMV_VBP	24
#define	LCD_TIMV_VFP	16
#define	LCD_TIMV_VSW	10
#define	LCD_TIMV_LPP	0

#define	LCD_POL		(*(volatile unsigned long *)(LCD_BASE + 0x008))
#define	LCD_POL_PCD_HI	27
#define	LCD_POL_BCD	26
#define	LCD_POL_CPL	16
#define	LCD_POL_IOE	14
#define	LCD_POL_IPC	13
#define	LCD_POL_IHS	12
#define	LCD_POL_IVS	11
#define	LCD_POL_ACB	6
#define	LCD_POL_CLKSEL	5
#define	LCD_POL_PCD_LO	0

#define	LCD_LE		(*(volatile unsigned long *)(LCD_BASE + 0x00c))
#define	LCD_LE_LEE	16
#define	LCD_LE_LED	0

#define	LCD_UPBASE	(*(volatile unsigned long *)(LCD_BASE + 0x010))
#define	LCD_LPBASE	(*(volatile unsigned long *)(LCD_BASE + 0x014))

#define	LCD_CTRL	(*(volatile unsigned long *)(LCD_BASE + 0x018))
#define	LCD_CTRL_WATERMARK	16
#define	LCD_CTRL_LCDVCOMP	12
#define	LCD_CTRL_LCDPWR		11
#define	LCD_CTRL_BEPO		10
#define	LCD_CTRL_BEBO		9
#define	LCD_CTRL_BGR		8
#define	LCD_CTRL_LCDDUAL	7
#define	LCD_CTRL_LCDMONO8	6
#define	LCD_CTRL_LCDTFT		5
#define	LCD_CTRL_LCDBW		4
#define	LCD_CTRL_LCDBPP		1
#define	LCD_CTRL_LCDEN		0

#define	LCD_INTMSK	(*(volatile unsigned long *)(LCD_BASE + 0x01c))
#define	LCD_INTMSK_BERIM	4
#define	LCD_INTMSK_VCOMPIM	3
#define	LCD_INTMSK_LNBUIM	2
#define	LCD_INTMSK_FUFIM	1

#define	LCD_INTRAW	(*(volatile unsigned long *)(LCD_BASE + 0x020))
#define	LCD_INTRAW_BERRAW	4
#define	LCD_INTRAW_VCOMPRIS	3
#define	LCD_INTRAW_LNBURIS	2
#define	LCD_INTRAW_FUFRIS	1

#define	LCD_INTSTAT	(*(volatile unsigned long *)(LCD_BASE + 0x024))
#define	LCD_INTSTAT_BERMIS	4
#define	LCD_INTSTAT_VCOMPMIS	3
#define	LCD_INTSTAT_LNBUMIS	2
#define	LCD_INTSTAT_FUFMIS	1

#define	LCD_INTCLR	(*(volatile unsigned long *)(LCD_BASE + 0x028))
#define	LCD_INTCLR_BERIC	4
#define	LCD_INTCLR_VCOMPIC	3
#define	LCD_INTCLR_LNBUIC	2
#define	LCD_INTCLR_FUFIC	1

/* the cursor bitfields are left for a later date as we do not need them now */
#define	LCD_UPCURR	(*(volatile unsigned long *)(LCD_BASE + 0x02c))
#define	LCD_LPCURR	(*(volatile unsigned long *)(LCD_BASE + 0x030))
#define	LCD_PAL		(*(volatile unsigned long *)(LCD_BASE + 0x200))
#define	CRSR_IMG	(*(volatile unsigned long *)(LCD_BASE + 0x800))
#define	CRSR_CTRL	(*(volatile unsigned long *)(LCD_BASE + 0xc00))
#define	CRSR_CFG	(*(volatile unsigned long *)(LCD_BASE + 0xc04))
#define	CRSR_PAL0	(*(volatile unsigned long *)(LCD_BASE + 0xc08))
#define	CRSR_PAL1	(*(volatile unsigned long *)(LCD_BASE + 0xc0c))
#define	CRSR_XY		(*(volatile unsigned long *)(LCD_BASE + 0xc10))
#define	CRSR_CLIP	(*(volatile unsigned long *)(LCD_BASE + 0xc14))
#define	CRSR_INTMSK	(*(volatile unsigned long *)(LCD_BASE + 0xc20))
#define	CRSR_INTCLR	(*(volatile unsigned long *)(LCD_BASE + 0xc24))
#define	CRSR_INTRAW	(*(volatile unsigned long *)(LCD_BASE + 0xc28))
#define	CRSR_INTSTAT	(*(volatile unsigned long *)(LCD_BASE + 0xc2c))
