/****************************************************************************
 *
 * Project: Framebuffer test application.
 *
 * Copyright by Olimex Ltd. All rights reserved.
 *
 * File: fbtest.c
 * Description: Very simple user-space Linux framebuffer test application.
 * Developer: Dimitar Dimitrov ( dinuxbg,gmail.com )
 *
 * Last change: $Date: 2008-09-29 14:28:41 +0300 (Mon, 29 Sep 2008) $
 * Revision: $Revision: 421 $
 * Id: $Id: fbtest.c 421 2008-09-29 11:28:41Z dimitar $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
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
 *
 ****************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include <errno.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <linux/fb.h>


/** 
 * The mmap-ed access to the framebuffer device file should be much
 * faster but unfortunately mmap does not work on all drivers. 
 */
#define CONFIG_MMAP	0

struct fb_sess {
	int fd;
	unsigned char *memp;
	struct fb_var_screeninfo vinfo;
	struct fb_fix_screeninfo finfo;
};


void fb_sync(struct fb_sess *fb);

static unsigned int fb_get_fb_size(struct fb_sess *fb)
{
	return fb->vinfo.xres * fb->vinfo.yres * (fb->vinfo.bits_per_pixel/8);
}


struct fb_sess *fb_init(const char *fb_dev_name)
{
	struct fb_sess *fb;
	int st, size;

	fb = malloc(sizeof(*fb));
	if(!fb) abort();

	fb->fd = open(fb_dev_name, O_RDWR);
	if(fb->fd < 0) {
		free(fb);
		fprintf(stderr, "Cannot open framebuffer device file.\n");
		exit(EXIT_FAILURE);
	}

	st = ioctl(fb->fd, FBIOGET_FSCREENINFO, &fb->finfo);
	if(st) {
		perror("ioctl failed");
		close(fb->fd);
		free(fb);
		fprintf(stderr, "Cannot get fixed screen info.\n");
		exit(EXIT_FAILURE);
	}

	st = ioctl(fb->fd, FBIOGET_VSCREENINFO, &fb->vinfo);
	if(st) {
		perror("ioctl failed");
		close(fb->fd);
		free(fb);
		fprintf(stderr, "Cannot get variable screen info.\n");
		exit(EXIT_FAILURE);
	}
	
	size = fb_get_fb_size(fb);
#if CONFIG_MMAP
	fb->memp = mmap(0, size, PROT_READ|PROT_WRITE, MAP_SHARED, fb->fd, 0);
	if(fb->memp == MAP_FAILED) {
		perror("mmap failed");
		close(fb->fd);
		free(fb);
		fprintf(stderr, "Cannot mmap the framebuffer device.\n");
		exit(EXIT_FAILURE);
	}
#else
	fb->memp = malloc(size);
	if(!fb->memp) {
		abort();
	}
#endif

	/* clear display */
	memset(fb->memp, 0, size);
	fb_sync(fb);

	return fb;
}

void fb_close(struct fb_sess *fb)
{
	if(fb) {
#if CONFIG_MMAP
		munmap(fb->memp, fb_get_fb_size(fb));
#else
		free(fb->memp);
#endif
		close(fb->fd);
		free(fb);
	}
}



/**
 * @note The 16 bit pixel color is formed from the following bits:
 * 	- bits 12-15: blue;
 * 	- bits 8-11: green;
 * 	- bits 4-7: red;
 * 	- bits 0-3: ???;
 */
void fb_draw_pixel(struct fb_sess *fb, unsigned int x, unsigned int y, unsigned int color)
{
	unsigned int offset;
	
	/* framebuffer is 16 bits per pixel */
	offset = (x + fb->vinfo.xoffset) * (fb->vinfo.bits_per_pixel/8) 
			+ (y + fb->vinfo.yoffset) * fb->finfo.line_length;
	switch(fb->vinfo.bits_per_pixel) {
		case 32:
		case 24:
			*(fb->memp + offset+2) = (color >> 16) & 0xff;
		case 16:
			*(fb->memp + offset+1) = (color >> 8) & 0xff;
		case 8:
			*(fb->memp + offset) = (color >> 0) & 0xff;
			break;
		default:
			abort();
	}
}

unsigned int fb_xres(struct fb_sess *fb)
{
	return fb->vinfo.xres;
}

unsigned int fb_yres(struct fb_sess *fb)
{
	return fb->vinfo.yres;
}

unsigned int fb_bits_per_pixel(struct fb_sess *fb)
{
	return fb->vinfo.bits_per_pixel;
}


void fb_sync(struct fb_sess *fb)
{
#if CONFIG_MMAP
	msync(fb->memp, fb_get_fb_size(fb), MS_SYNC);
#else
	lseek(fb->fd, 0, SEEK_SET);
	write(fb->fd, fb->memp, fb_get_fb_size(fb));
#endif
}


void draw_sine(struct fb_sess *fb)
{
	double res;
	unsigned int x;
	const unsigned int color = 0xff00;

	for(x=0; x < fb_xres(fb); ++x) {
		unsigned int y;

		res = sin((double)x / ((double)fb_xres(fb)/32));
		res = res * ((double)fb_yres(fb) - 10) / 2;
		y = fb_yres(fb)/2 + (int)res;
		if((y+5) > fb_yres(fb)) {
			y = fb_yres(fb)-1;
		}
		if(y < 5) {
			y = 5;
		}

		fb_draw_pixel(fb, x, y, color);
		fb_draw_pixel(fb, x, y+1, color);
		fb_draw_pixel(fb, x, y-1, color);
		fb_draw_pixel(fb, x+1, y, color);
		fb_draw_pixel(fb, x+1, y+1, color);
		fb_draw_pixel(fb, x+1, y-1, color);
		fb_draw_pixel(fb, x-1, y, color);
		fb_draw_pixel(fb, x-1, y+1, color);
		fb_draw_pixel(fb, x-1, y-1, color);
	}
	fb_sync(fb);
}



void draw_lines(struct fb_sess *fb)
{
	unsigned int x, y;

	for(x=0; x<fb_xres(fb); ++x) {
		for(y=0; y<fb_yres(fb); ++y) {
			fb_draw_pixel(fb, x, y, 2);
		}
	}

	for(x=0; x<fb_xres(fb); ++x) {
		fb_draw_pixel(fb, x, x % fb_yres(fb), 0xff0000);
		fb_draw_pixel(fb, x, (fb_xres(fb)-x) % fb_yres(fb), 0xff);
	
	}
	fb_sync(fb);
}



#define ARRAY_NUMELEM(A)	((sizeof(A)/sizeof((A)[0])))
static void memtest(void)
{
	uint32_t *mem, *p;
	unsigned int i;
	const int size_mb = 4;
	static char progress[] = {'-', '\\', '|', '/'};

	printf("\n\n>>>>>>>>>>>>>>>>MEMORY-TEST<<<<<<<<<<<<<<<<\n");

	mem = malloc(size_mb*1024*1024);
	if(!mem) {
		printf("Could not allocate a %d megabyte chunk.\n",size_mb);
		return;
	}

	printf("Testing %d megabytes memory at %p\n", size_mb, mem);

	srand(1);
	p = mem;
	for(i=0; i<size_mb*1024/4; ++i) {
		unsigned int j;

		for(j=0; j<1024; ++j) {
			uint32_t val = (rand()&0xFFFF) | (rand()&0xffff)<<16;
			*p++ = val;
		}

		printf("\rWrote %d/%d kilobytes...\t%c    ", 
				(i+1)*4, size_mb*1024,
				progress[i % (ARRAY_NUMELEM(progress))]);
		fflush(stdout);
	}

	printf("\n");
	srand(1);
	p = mem;
	for(i=0; i<size_mb*1024/4; ++i) {
		unsigned int j;

		for(j=0; j<1024; ++j) {
			uint32_t r = *p++;
			uint32_t val = (rand()&0xFFFF) | (rand()&0xffff)<<16;
			if(r != val) {
				printf("Mismatch: read=%08x, expected=%08x\n",
						r, val);
				free(p);
				return;
			}
		}
		printf("\rVerified %d/%d kilobytes...\t%c      ", 
				(i+1)*4, size_mb*1024,
				progress[i % (ARRAY_NUMELEM(progress))]);
		fflush(stdout);
	}
	free(p);
	printf("\nMemory test completed.       \n");
	fflush(stdout);
}


int main(int argc, char *argv[])
{
	struct fb_sess *fb;
	const char *fb_dev_name;
	char buf[256];

	if(argc == 1) {
		fb_dev_name = "/dev/fb0";
	} else if(argc == 2) {
		fb_dev_name = argv[1];
	} else {
		fprintf(stderr, "Usage: \n\t%s [FRAMEBUFFER_DEVICE]\n\n",
				argc ? argv[0] : "fbtest");
		exit(EXIT_FAILURE);
	}

	printf("Trying to open the framebuffer device...\n");

	fb = fb_init(fb_dev_name);
	if(!fb) {
		fprintf(stderr, "Could not initialize %s.\n", fb_dev_name);
		exit(EXIT_FAILURE);
	}

	printf("Getting framebuffer information:\n");
	printf("\tID:            %s\n", fb->finfo.id);
	printf("\tcolordepth:    %d bits/pixel\n", fb_bits_per_pixel(fb));
	printf("\tvideo memsize: %d bytes\n", fb_get_fb_size(fb));
	printf("\tresolution:    %dx%d\n", fb_xres(fb), fb_yres(fb));
	printf("\tvirtual res.:  %dx%d\n", fb->vinfo.xres_virtual,
					   fb->vinfo.yres_virtual);
	printf("\tgrayscale:     %d\n", fb->vinfo.grayscale);
	printf("\tphys size:     %dx%d mm\n", fb->vinfo.width,
						fb->vinfo.height);
	printf("\tpixel clock:   %d picoseconds\n", fb->vinfo.pixclock);
	printf("\tleft margin:   %d\n", fb->vinfo.left_margin);
	printf("\tright margin:  %d\n", fb->vinfo.right_margin);
	printf("\tupper margin:  %d\n", fb->vinfo.upper_margin);
	printf("\tlower margin:  %d\n", fb->vinfo.lower_margin);
	printf("\thsync_len:     %d\n", fb->vinfo.hsync_len);
	printf("\tvsync_len:     %d\n", fb->vinfo.vsync_len);
	printf("\trotate:        %d\n", fb->vinfo.rotate);
	printf("\tsmem_start:    0x%08x\n", fb->finfo.smem_start);
	printf("\tsmem_len:      %d\n", fb->finfo.smem_len);
	printf("\tline_length:   %d\n", fb->finfo.line_length);
	printf("\tmmio_start:    0x%08x\n", fb->finfo.mmio_start);
	printf("\tmmio_len:      %d\n", fb->finfo.mmio_len);
	printf("\tmmio_len:      %d\n", fb->finfo.mmio_len);


	draw_lines(fb);
	usleep(1000*1000);
	draw_sine(fb);

	fb_close(fb);

	if(1) {
		memtest();
	}

	printf("Done. Exitting...\n");

	return EXIT_SUCCESS;
}


