#include "touch.h"
#include "test"

unsigned int touch_detect (void){
	config_pins_touch ();				// configure pins for touch detection
	timer_delay (settling);				// settling time for switching
	return((IOPIN0 & X_plus)^X_plus); 	// return true if touch is detected
}

void detected(void) __irq	{
	short x_value, y_value, i;
	timer_delay (debounce);					// debounce the touch
	while (touch_detect())	{				// loop as long as screen is touched
		led_red();
		read_ch_x();						// read and collect the x values
		read_ch_y();						// read and collect the y values

		x_value = 0;						// initial value
		for (i=0; i < num_samples; i++)	{
			x_value += x_values[i];			// add up the conversion results
		}

		x_value = x_value /num_samples;		// get average
		y_value = 0;
		for (i=0; i < num_samples; i++){	// initial value
			y_value += y_values[i];			// add up conversion results
		}
		y_value = y_value /num_samples;		// get average



		if (touch_detect())display_lcd(x_value, y_value); 	// display values if
															// still have a touch
	}
IO0_INT_CLR = X_plus;	// clear X-plus interrupt
led_green();
VICVectAddr = 0;	    // Acknowledge Interrupt
}

void led_green (void)
{
	IOSET0 |= 0x00000800; // P0.11 = high
	IOCLR0 |= 0x00000400; // P0.10 = low
}

void led_red (void)
{
	IOSET0 |= 0x00000400; // P0.10 = high
	IOCLR0 |= 0x00000800; // P0.11 = low
}

static void initialisation(void){
	PCONP |= (1 << 12); 					// Enable power to AD block
	for (i = 0; i < 20000000; i++); 		// Wait for initial display
	IODIR0 |= 0x00000C00; 					// config touch LED pins as outputs
	led_green(); 							// make the LED green

	PINMODE4 &= ~(0xFFFF); 					// P2[7:0]pullups
	PINSEL4 &= ~(0xFFFF); 					// P2[7:0]are GPIO
	FIO2DIR0 = 0xFF; 						// P2[7:0]are outputs
	FIO2MASK0 = 0x00; 						// P2[7:0]enabled for fast I/O

	touch_detect();							// setup for touch detection
	install_irq(17, (void*)detected, 1); 	// setup interrupt vector
	IO0_INT_EN_F = X_plus;					// enable falling edge X-plus interrupt
}

void config_pins_x (void){
	PINSEL0  &= ~(X_minus_mask);
	PINMODE0 &= ~(X_minus_mask);
	PINMODE0 |= X_minus_no_pull;
	IODIR0   |= X_minus;
	IOCLR0   |= X_minus;

	PINSEL0  &= ~(Y_minus_mask);
	PINMODE0 &= ~(Y_minus_mask);
	PINMODE  |= Y_minus_no_pull;
	IODIR0   &= ~(Y_minus); 

	PINSEL1  &= ~(X_plus_mask);		// no pullup on X+
	PINMODE1 &= ~(X_minus_mask); 	// X+ is an output
	PINMODE1 |= X_plus_no_pull; 	// make X+ high
 	IODIR0   |= X_plus;
	IOSET0   |= X_plus; 

	PINSEL1 &= ~(Y_plus_mask);
  	PINSEL1 |= ADC_on_Y;			// Y+ is an ADC pin
}

void config_pins_y (void){

	PINSEL0 &= ~(X_minus_mask); 
	PINMODE0 &= ~(X_minus_mask);
	PINMODE0 |= X_minus_no_pull; 
	IODIR0 &=  ~(X_minus); 

	PINSEL0 &= ~(Y_minus_mask); // make Y- low
	PINMODE0 &= ~(Y_minus_mask);
	PINMODE0 |= Y_minus_no_pull; 
	IODIR0 |= Y_minus; 
	IOCLR0 |= Y_minus;
		           
	PINSEL1 &= ~(X_plus_mask);// X+ is an ADC pin
	PINSEL1 |= ADC_on_X;

	PINSEL1 &= ~(Y_plus_mask);
	PINMODE1 &= ~(Y_plus_mask); 
	PINMODE1 |=  Y_plus_no_pull; 
	IODIR0  |=  Y_plus; 
	IOSET0 |=  Y_plus; 
}

void config_pins_touch (void)
{
	PINSEL0 &= ~(X_minus_mask);//
	PINMODE0 &= ~(X_minus_mask); //
	PINMODE0 |=  X_minus_no_pull;//
	IODIR0 &= ~(X_minus); 
	PINSEL0 &= ~(Y_minus_mask); // Y- is digital I/O
	PINMODE0 &= ~(Y_minus_mask); 
	PINMODE0 |= Y_minus_no_pull;  
	IODIR0  |= Y_minus; 
	IOCLR0 |= Y_minus;        
	PINSEL1 &= ~(Y_plus_mask);
	PINMODE1 &= ~(Y_plus_mask); 
	PINMODE1 |= Y_plus_no_pull; 
	IODIR0 &=  ~(Y_plus); 
	PINSEL1 &= ~(X_plus_mask);
	PINMODE1 &= ~(X_plus_mask);
	IODIR0 &= ~(X_plus);
}
