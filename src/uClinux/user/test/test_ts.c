#include <linux/input.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#define MAX_GLYPHS 100

#define XRES 320
#define YRES 240

/*
 * Global variable
 */
unsigned short face_size;


struct Rect {
	int x;
	int y;
	int w;
	int h;
};

#define GET_MEMORY_ADRESS _IOW('G', 1, void*)
int* fbp;

void draw_button(int x, int y, int w, int h, int color) {
	unsigned short i, j;
	for(j = y; j < y+h; j++) {
		for(i = x; i < x+w; i++) {
			fbp[j * XRES + i] = color;
		}
	}
}

int main(int argc, char **argv)
{
	int fb = open("/dev/fb0", O_RDWR);
	int input = open("/dev/input/event0", O_RDONLY);
	struct input_event ev;
	int i, j;
	int x, y;
	struct Rect rect;
	int color_rect = 0x00FF00;

	rect.x = rect.y = 50;
	rect.w = 50;
	rect.h = 70;

	ioctl(fb, GET_MEMORY_ADRESS, &fbp);


	// Clean the screen
	for(i = 0; i < YRES; i++)
		for(j = 0; j < XRES; j++)
			fbp[i * XRES + j] = 0xFF00FF;

	// main loop
	while(1)
	{
		// Poll event
		read(input, &ev, sizeof(struct input_event));

		if(ev.code == ABS_PRESSURE) {
			if(ev.value == 1) {
				if(x > rect.x && x < rect.x + rect.w) {
					if(y > rect.y && y < rect.y + rect.h) {
						if(color_rect == 0x00FF00)
							color_rect = 0x0000FF;
						else
							color_rect = 0x00FF00;
					}
				}

				// Quit !
				if(x > 300)
					if(y > 200)
						return 0;
			}
		}

		if(ev.code == ABS_X) {
			x = ev.value;
		}

		if(ev.code == ABS_Y) {
			y = ev.value;
		}

		// Draw the screen
		draw_button(rect.x, rect.y, rect.w, rect.h, color_rect);
	}
}
