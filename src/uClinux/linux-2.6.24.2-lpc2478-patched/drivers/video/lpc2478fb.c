/****************************************************************************
 *  
 * Project: uClinux support for LPC-2478-STK
 *
 * Copyright: Ivan Vasilev, Olimex Ltd. All rights reserved.
 * 
 * File: $File lpc2478fb.c $
 * Description: The framebuffer driver. Based on drivers/video/skeletonfb.c
 * Developer: Ivan Vasilev, <ivan at l123.org>
 *
 * Last change: $Date: 2008-04-08 09:23:26 +0300 (вторник, 08 Април 2008) $
 * Revision: $Revision: 4 $
 * Id: $Id: lpc2478fb.c  4 2008-04-08 06:23:26Z Ivan $
 * Author: $Author: Ivan $
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 ****************************************************************************/
 
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/reboot.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "lpc2478fb.h"
#include "lpc2468_registers.h"
/*#include <asm/hardware.h>*/
#include <linux/platform_device.h>

#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "console/fbcon.h"

#define DEBUG 1
#ifdef DEBUG
#define DPRINTK( fmt, arg... )  printk( fmt, ##arg )
#else
#define DPRINTK( fmt, arg... )
#endif

#define	SCREEN_SIZE_X	320
#define	SCREEN_SIZE_Y	240
#define	FBMEM_SIZE	(SCREEN_SIZE_X * SCREEN_SIZE_Y * 4)

static int __init lpc2478fb_probe(struct platform_device *dev);
int __init lpc2478fb_init(void);
static int lpc2478_fb_mmap(struct fb_info *info, struct vm_area_struct *vma);

enum _COLOR_MODE
{
	COLOR_MODE_1BPP,
	COLOR_MODE_2BPP,
	COLOR_MODE_4BPP,
	COLOR_MODE_8BPP,
	COLOR_MODE_16BPP,
	COLOR_MODE_24BPP,
	COLOR_MODE_16BPP_565,
	COLOR_MODE_12BPP_444
};
typedef enum _COLOR_MODE COLOR_MODE;

typedef struct _LCD_INIT_STRUCTURE
{
	uint16_t	xsize;
	uint8_t		horiz_front_porch;
	uint8_t		horiz_back_porch;
	uint8_t		horiz_sync_pulse;
	uint16_t	ysize;
	uint8_t		vert_front_porch;
	uint8_t		vert_back_porch;
	uint8_t		vert_sync_pulse;
	COLOR_MODE	color_mode;
	uint8_t		hclk_divider;
} LCD_INIT_STRUCTURE;

static struct fb_fix_screeninfo lpc2478fb_fix __initdata = {
	.id		= "LPC-2478 LCD",
	.smem_len	= FBMEM_SIZE,
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_TRUECOLOR,
	.line_length	= SCREEN_SIZE_X * 4,
	.accel		= FB_ACCEL_NONE,
};

static struct fb_var_screeninfo lpc2478fb_var __initdata = {
	.xres		= 320,
	.yres		= 240,
	.xres_virtual	= 320,
	.yres_virtual	= 240,
	.bits_per_pixel	= 32,
    	.red		= {0, 8, 0},
	.green		= {8, 8, 0},
	.blue		= {16, 8, 0},
	.activate	= FB_ACTIVATE_NOW,
	.pixclock 	= 3600000,
      	.vmode		= FB_VMODE_NONINTERLACED,
};


#if 0
static struct fb_ops lpc2478fb_ops = {
	.owner		= THIS_MODULE,
	.fb_read        = fb_sys_read,
	.fb_write       = fb_sys_write,
	.fb_fillrect	= sys_fillrect,
	.fb_copyarea	= sys_copyarea,
	.fb_imageblit	= sys_imageblit,
	//.fb_fillrect	= cfb_fillrect,
	//.fb_copyarea	= cfb_copyarea,
	//.fb_imageblit	= cfb_imageblit,
};
#else
static struct fb_ops lpc2478fb_ops = 
{
	.owner			= THIS_MODULE,
	//.fb_mmap 		= lpc2478_fb_mmap,
	.fb_fillrect		= cfb_fillrect,
	.fb_copyarea		= cfb_copyarea,
	.fb_imageblit		= cfb_imageblit,
};
#endif

/* this is the actual physical start address of our framebuffer */
static dma_addr_t	fb_phys;
static void * 		fb_log;
static uint32_t fb_smem_start;


static int lpc2478_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{

	/* stolen from bf54x-lq043fb.c */

	//vma->vm_start = (unsigned long)(fbi->fb_buffer);
	vma->vm_start = fb_smem_start;

	vma->vm_end = vma->vm_start + info->fix.smem_len;
	/* For those who don't understand how mmap works, go read
	 *   Documentation/nommu-mmap.txt.
	 * For those that do, you will know that the VM_MAYSHARE flag
	 * must be set in the vma->vm_flags structure on noMMU
	 *   Other flags can be set, and are documented in
	 *   include/linux/mm.h
	 */
	vma->vm_flags |= VM_MAYSHARE;

	return 0;
}

static struct platform_driver lpc2478fb_driver = {
	.probe	= lpc2478fb_probe,
	.driver	= {
		.name	= "lpc2478fb",
	},
};

static struct platform_device lpc2478fb_device = {
	.name	= "lpc2478fb",
};

/* these two functions are taken from the ep93xx framebuffer driver */
static int lpc2478fb_alloc_videomem(void)
{
	//unsigned long test = 0;
    //unsigned long adr,size,pgsize,cntr;
    unsigned long adr,size;//,pgsize,cntr;
    unsigned long * ptr;
    //int order;
    int col, row;
    void *mem;
    
    size = FBMEM_SIZE;    

    DPRINTK("lpc2478fb_alloc_videomem - enter \n");

    /*fb_log = NULL;
    lpc2478fb_fix.smem_len = PAGE_ALIGN( FBMEM_SIZE );
    order = get_order( lpc2478fb_fix.smem_len );
    fb_log = (void *)__get_free_pages( GFP_KERNEL, order );
    
    if (fb_log)
    {
	fb_phys = __virt_to_phys((int)fb_log);
	adr = (unsigned long)fb_log;
	size = lpc2478fb_fix.smem_len;
	pgsize = 1<<order;
	do
	{
		SetPageReserved(virt_to_page(adr));
		memset(adr, test, pgsize);
		test += 0x04;
		adr += pgsize;
	} while (size -= pgsize);
    }
    //memset(fb_log, 0x00000005, lpc2478fb_fix.smem_len);*/
	
	size = PAGE_ALIGN(size);
	mem = vmalloc_32(size);
	//mem = (void*)0xa1f00000;
	if (!mem)
		return -ENOMEM;

	memset(mem, 0, size); /* Clear the ram out, no junk to the user */
	adr = (unsigned long) mem;
	while (size > 0) {
		SetPageReserved(vmalloc_to_page((void *)adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
    fb_log = mem;
    fb_phys = (uint32_t)mem;
    lpc2478fb_fix.smem_start = fb_smem_start = (uint32_t)mem;
    lpc2478fb_fix.smem_len = PAGE_ALIGN(FBMEM_SIZE);
    
    size = 0;
    ptr = mem;
    for (row = 0; row < 240; row ++)
    {
	for ( col =0; col < 320; col++)
    	{
		if (col < 107) *ptr++ = 0x00ff0000;
		else if (col < 214) *ptr++ = 0x0000ff00;
		else *ptr++ = 0x000000ff;
	}
    }
    
    /*for (ptr = mem; ptr < (PAGE_ALIGN(FBMEM_SIZE)+mem); ptr++)
    {
	*ptr = 0x000000ff;
	// *ptr = size;
	//size += 8;
	//for (cntr = 1000; cntr; cntr--);
    }*/
    
    DPRINTK("   fb_log_addres = 0x%x\n",(uint32_t)fb_log);
    DPRINTK("   fb_phys_address = 0x%x\n",(uint32_t)fb_phys);
    DPRINTK("   fb_size = %lu\n",(unsigned long)lpc2478fb_fix.smem_len);
    //DPRINTK("   fb_page_order = %d\n",order);
    DPRINTK("lpc2478fb_alloc_videomem - exit \n");
    return 0;       
}

#if 0
static void lpc2478fb_release_videomem(void)
{
    //unsigned long adr,size,psize;
    //int order;
    	
    DPRINTK("lpc2478fb_release_videomem - enter \n");
    DPRINTK("lpc2478fb_release_videomem not actually implemented for now!!! \n");
    DPRINTK("lpc2478fb_release_videomem - exit \n");
}
#endif

void lpc2478fb_hwinit(LCD_INIT_STRUCTURE lcd)
{
	PCONP |= 1<<20;		/* Turn On LCD PCLK */
	
	PINSEL0 &= 0xfffc00ff;//BIN32(11111111,11111100,00000000,11111111);
	PINSEL0 |= 0x00015500;//BIN32(00000000,00000001,01010101,00000000);
	PINMODE0&= 0xfffc00ff;//BIN32(11111111,11111100,00000000,11111111);
	PINMODE0|= 0x0002aa00;//BIN32(00000000,00000010,10101010,00000000);
	PINSEL3 &= 0xf00000ff;//BIN32(11110000,00000000,00000000,11111111);
	PINSEL3 |= 0x05555500;//BIN32(00000101,01010101,01010101,00000000);
	PINMODE3&= 0xf00000ff;//BIN32(11110000,00000000,00000000,11111111);
	PINMODE3|= 0x0aaaaa00;//BIN32(00001010,10101010,10101010,00000000);
	PINSEL4 &= 0xf0300000;//BIN32(11110000,00110000,00000000,00000000);
	PINSEL4 |= 0x058fffff;//BIN32(00000101,01001111,11111111,11111111);
	PINMODE4&= 0xf0300000;//BIN32(11110000,00110000,00000000,00000000);
	PINMODE4|= 0x0a8aaaaa;//BIN32(00001010,10001010,10101010,10101010);
	PINSEL9 &= 0xf0ffffff;//BIN32(11110000,11111111,11111111,11111111);
	PINSEL9 |= 0x0a000000;//BIN32(00001010,00000000,00000000,00000000);
	PINMODE9&= 0xf0ffffff;//BIN32(11110000,11111111,11111111,11111111);
	PINMODE9|= 0x0a000000;//BIN32(00001010,00000000,00000000,00000000);
	PINSEL11&= 0xfffffff0;//BIN32(11111111,11111111,11111111,11110000);
	PINSEL11|= 0x0000000f;//BIN32(00000000,00000000,00000000,00001111);

	LCD_CFG = (uint32_t)lcd.hclk_divider << LCD_CFG_CLKDIV;

	LCD_TIMH = (	(((uint32_t)lcd.horiz_back_porch-1) << LCD_TIMH_HBP ) | \
		   	(((uint32_t)lcd.horiz_front_porch-1) << LCD_TIMH_HFP) | \
			(((uint32_t)lcd.horiz_sync_pulse-1) << LCD_TIMH_HSW ) | \
			(((lcd.xsize / 16) - 1) << LCD_TIMH_PPL ) );

	LCD_TIMV = (	(((uint32_t)lcd.vert_back_porch) << LCD_TIMV_VBP ) | \
			(((uint32_t)lcd.vert_front_porch) << LCD_TIMV_VFP) | \
			(((uint32_t)lcd.vert_sync_pulse) << LCD_TIMV_VSW ) | \
			(( lcd.ysize-1) << LCD_TIMV_LPP ) );

	LCD_POL = (	(((uint32_t)0	) << LCD_POL_PCD_HI	) | \
			(((uint32_t)1	) << LCD_POL_BCD	) | \
			(((uint32_t)lcd.xsize-1	) << LCD_POL_CPL	) | \
			(((uint32_t)0	) << LCD_POL_IOE	) | \
			(((uint32_t)1	) << LCD_POL_IPC	) | \
			(((uint32_t)1	) << LCD_POL_IHS	) | \
			(((uint32_t)1	) << LCD_POL_IVS	) | \
			(((uint32_t)0	) << LCD_POL_ACB	) | \
			(((uint32_t)0	) << LCD_POL_CLKSEL	) | \
			(((uint32_t)0	) << LCD_POL_PCD_LO	) );
			
	LCD_LE = 0;
	LCD_UPBASE = ((uint32_t)fb_phys);
	LCD_LPBASE = ((uint32_t)fb_phys); 

	LCD_CTRL = (	(((uint32_t) 1	) << LCD_CTRL_LCDPWR	) | \
			(((uint32_t) 1	) << LCD_CTRL_LCDTFT	) | \
			(((uint32_t)lcd.color_mode ) << LCD_CTRL_LCDBPP ) );
			
	LCD_CTRL |= 1 << LCD_CTRL_LCDEN;
}

static int __init lpc2478fb_probe(struct platform_device *dev)
{
	uint32_t *cntr;
	struct fb_info *info;
	
	LCD_INIT_STRUCTURE mylcd;
	mylcd.xsize = 320;
	mylcd.horiz_front_porch = 20;
	mylcd.horiz_back_porch = 38;
	mylcd.horiz_sync_pulse = 30;
	mylcd.ysize = 240;
	mylcd.vert_front_porch = 5;
	mylcd.vert_back_porch = 15;
	mylcd.vert_sync_pulse = 3;
	mylcd.color_mode = COLOR_MODE_24BPP;
	mylcd.hclk_divider = 15;

	lpc2478fb_alloc_videomem();
	
	info = framebuffer_alloc(sizeof(u32) * 16, &dev->dev);
	if (!info)
		return -ENOMEM;

	info->device = &dev->dev;
	info->var = lpc2478fb_var;
	info->fix = lpc2478fb_fix;
	info->fbops = &lpc2478fb_ops;
	info->flags = FBINFO_DEFAULT;  /* not as module for now */
	info->pseudo_palette = info->par;
	info->par = NULL;
	info->screen_base = (char *) lpc2478fb_fix.smem_start;

	if (fb_alloc_cmap(&info->cmap, 256, 0) < 0) {
		framebuffer_release(info);
		return -ENOMEM;
	}

	/*master_outb(3, DISPLAY_CONTROL_REG);*/

	if (register_framebuffer(info) < 0) {
		printk(KERN_ERR "Unable to register lpc2478 frame buffer\n");
		fb_dealloc_cmap(&info->cmap);
		framebuffer_release(info);
		return -EINVAL;
	}
	
	lpc2478fb_hwinit(mylcd);
	
#if !defined(CONFIG_FRAMEBUFFER_CONSOLE) && defined(CONFIG_LOGO)
	if (fb_prepare_logo(info, FB_ROTATE_UR)) {
		/* Start display and show logo on boot */
		//fb_set_cmap(info->cmap, info);
		fb_show_logo(info, FB_ROTATE_UR);
	}
#endif

        printk(KERN_INFO "fb%d: lpc2478 framebuffer initialized\n",
	       info->node);
	
	cntr = (uint32_t*)0xffe08100;
	DPRINTK("   0x%x = 0x%x\n",(uint32_t)cntr, *cntr);
	
	DPRINTK("   lpc2478_fbmem:\n");
	cntr = (uint32_t*)0xe01fc1b8;
	DPRINTK("   0x%x = 0x%x\n",(uint32_t)cntr, *cntr);
	cntr = (uint32_t*)0xffe10000;
	for (;cntr < (uint32_t*)0xffe10034; cntr++)
	{
		DPRINTK("   0x%x = 0x%x\n",(uint32_t)cntr, *cntr);
	}

	
	return 0;
}

int __init lpc2478fb_init(void)
{
	int ret = 0;

	if (fb_get_options("lpc2478fb", NULL))
		return -ENODEV;

	ret = platform_driver_register(&lpc2478fb_driver);

	if (!ret) {
		struct platform_device *dev;

		dev = platform_device_alloc("lpc2478fb", 0);

#if 0
		ret = platform_device_register(&lpc2478fb_device);
		if (ret)
			platform_driver_unregister(&lpc2478fb_driver);
#else
		if (dev)
			ret = platform_device_add(dev);
		else
			ret = -ENOMEM;

		if (ret) {
			platform_device_put(dev);
			platform_driver_unregister(&lpc2478fb_driver);
		}
#endif

	}
	return ret;
}

module_init(lpc2478fb_init);
MODULE_LICENSE("GPL");
