#ifndef UNDRIVER_H_
#define UNDRIVER_H_

#include <linux/ioctl.h>

#define MAXDEVICES 1
#define DRIVERNAME "ts"

#endif

char* device = "ts";

unsigned int touch_detect (void);
void detected(void) __irq ;
void led_green (void) ;
void led_red (void) ;
static void initialisation(void) ;
void config_pins_x (void) ;
void config_pins_y (void) ;
void config_pins_touch (void) ;
