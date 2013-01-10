#include <stdio.h>
#include "lpc24xx.h"
#include "lpc_types.h"


/******************************************************************************/
/*                                     pin definitions                          */
/******************************************************************************/
#define  X_plus 0x01000000                    // X+ on P0.24
#define  X_plus_mask 0x00030000               // X+ pin select mask (ADC0.1)
#define  X_plus_no_pull 0x00020000            // X+ no pullup value
#define  ADC_on_X 0x00010000                  // X+ pin select (1) ADC
#define X_minus 0x00000100                    // X- on P0.8
#define X_minus_mask 0x00010000               // X- pin select mask
#define X_minus_no_pull 0x00020000            // X- no pullup value
#define  Y_plus 0x02000000                    // Y+ on P0.25
#define  Y_plus_mask 0x000C0000               // Y+ pin select mask (ADC0.2)
#define  Y_plus_no_pull 0x00080000            // Y+ no pullup value
#define  ADC_on_Y 0x00040000                  // Y+ pin select (1) ADC
#define Y_minus 0x00000200                    // Y- on P0.9
#define Y_minus_mask 0x000C0000               // Y- pin select mask
#define Y_minus_no_pull 0x00080000            // Y- no pullup value
/******************************************************************************/
/*                                 timer count definitions       */
/******************************************************************************/
#define debounce 1000                         // debounce delay
#define settling 100                          // settling time delay
/******************************************************************************/
/*                                 function prototypes                         */
/******************************************************************************/
#define VECT_ADDR_INDEX	0x100
#define VECT_CNTL_INDEX 0x200

unsigned long install_irq( unsigned long IntNumber, void *HandlerAddr, unsigned long Priority ) {
	int *vect_addr;
	int *vect_cntl;

	VICIntEnClr = 1 << IntNumber;	/* Disable Interrupt */
	VICIntEnClr = 1 << IntNumber;	/* Disable Interrupt */
	/* find first un-assigned VIC address for the handler */
	vect_addr = (int *)(VIC_BASE_ADDR + VECT_ADDR_INDEX + IntNumber*4);
	vect_cntl = (int *)(VIC_BASE_ADDR + VECT_CNTL_INDEX + IntNumber*4);
	*vect_addr = (int)HandlerAddr;	/* set interrupt vector */
	*vect_cntl = Priority;
	VICIntEnable = 1 << IntNumber;	/* Enable Interrupt */
	return 1;
}

void config_pins_x (void);

void config_pins_y (void);
void config_pins_touch (void);
void detected(void);
void display_lcd(short x_value, short y_value);
char hex_to_ascii(char ch);
void read_ch_x (void);
void read_ch_y (void);
void timer_delay (unsigned int count);
unsigned int touch_detect (void);
/******************************************************************************/
/*                                     globals                                   */
/******************************************************************************/
#define num_samples 16                       // number of A/D samples per axis
unsigned int x_values[num_samples];          // array to store x_samples
unsigned int y_values[num_samples];          // array to store y_samples
/******************************************************************************/
/*                                     start of main code       */
/******************************************************************************/

#define SAMPLES 32

int main (void)
{
	int X = 0;
	int Y = 0;
	int X_temp, Y_temp;
	int i, j;
	PCONP |= (1 << 12);      // Enable power to AD block

	SCS |= 1; // Enable Fast GPIO

	PINSEL1 &= ~(3 << 16);   // reset X1
	PINSEL1 |= (1 << 16);    // X1 => 01 P0..24
	PINSEL1 &= ~(3 << 12);   // X2 => 00 P0..22
	PINSEL1 &= ~(3 << 14);   // Y1 => 00 P0..23
	PINSEL1 &= ~(3 << 10);   // Y2 => 00 P0..21

	PINMODE1 &= ~(3 << 16);
	PINMODE1 |= (2 << 16);   // X1 => no 10
	PINMODE1 &= ~(3 << 12);
	PINMODE1 |= (3 << 12);   // X2 => enable pull-down
	PINMODE1 &= ~(3 << 14);
	PINMODE1 |= (2 << 14);   // Y1 => no 10
	PINMODE1 &= ~(3 << 10);
	PINMODE1 |= (2 << 10);   // Y2 => no 10

	FIO0DIR   &=  ~(1 << 24);     // X- is an input
	FIO0DIR   &=  ~(1 << 22);     // X- is an input
	FIO0DIR   |=   (1 << 23);     // Y- is an output
	FIO0DIR   |=   (1 << 21);     // Y- is an output

	FIO0SET   =   (1 << 23);      // Y- is an output
	FIO0SET   =   (1 << 21);      // Y- is an output

	// Init ADC
	AD0CR |= (1 << 21); // PDN
	AD0CR &= ~(1 << 24); // START
	AD0CR |= (1 << 1); // Ch1
	AD0CR &= ~(0xFF << 8);
	AD0CR |= (8 << 8); // Div
	AD0CR &= ~(1 << 16); // BURST
	AD0CR &= ~(1 << 16); // CLOCK
	AD0CR &= ~(7 << 17); // CLKS

	while(1) {
		for (i = 0; i < 200000; i++);     // delay
		if(FIO0PIN & (1 << 22)) { // Touch detected
			X = Y = 0;

			// Y1 = 0, Y2 = 1;
			FIO0CLR = (1UL << 23);
			// Disable X2 pull down
			PINMODE1 &= ~(3 << 12);
			PINMODE1 |= (2 << 12);   // X1 => no 10

			//
			// X1 MESURE
			//
			for(j = 0; j < SAMPLES; j++) {
				AD0CR |= (1 << 24); // START
				while(!(AD0GDR)&(1 << 31)); // Wait for completion
				AD0CR &= ~(1 << 24); // START
				Y += (AD0GDR >> 6) & 0x3FF;
			}

			// Y2 = 0, Y1 = 1;
			FIO0CLR = (1 << 21);
			FIO0SET = (1 << 23);

			//
			// X2 MESURE
			//
			for(j = 0; j < SAMPLES; j++) {
				AD0CR |= (1 << 24); // START
				while(!(AD0GDR)&(1 << 31)); // Wait for completion
				AD0CR &= ~(1 << 24); // START
				Y += 1023 - ((AD0GDR >> 6) & 0x3FF);
			}

			// X1 = 0, X2 = 1;
			FIO0CLR  = (1 << 24);
			FIO0SET  = (1 << 22);
			FIO0DIR |= (1 << 24);
			FIO0DIR |= (1 << 22);
			PINSEL1 &= ~(3 << 16);   // reset X1

			// Y1 - ADC Ch0, Y2 input
			FIO0DIR &= ~(1 << 23);
			FIO0DIR &= ~(1 << 21);
			PINSEL1 &= ~(3 << 14); 
			PINSEL1 |= (1 << 14); 
			AD0CR &= ~(3);
			AD0CR |= (1 << 0); // Ch0

			//
			// Y1 MESURE
			//
			for(j = 0; j < SAMPLES; j++) {
				AD0CR |= (1 << 24); // START
				while(!(AD0GDR)&(1 << 31)); // Wait for completion
				AD0CR &= ~(1 << 24); // START
				X += 1023 - ((AD0GDR >> 6) & 0x3FF);
			}

			// X2 = 0, X1 = 1;
			FIO0CLR = (1 << 22);
			FIO0SET = (1 << 24);

			//
			// Y2 MESURE
			//
			for(j = 0; j < SAMPLES; j++) {
				AD0CR |= (1 << 24); // START
				while(!(AD0GDR)&(1 << 31)); // Wait for completion
				AD0CR &= ~(1 << 24); // START
				X += (AD0GDR >> 6) & 0x3FF;
			}

			// Y1 = 1, Y2 = 1;
			FIO0SET = (1 << 23);
			FIO0SET = (1 << 21);
			FIO0DIR |= (1 << 23);
			FIO0DIR |= (1 << 21);
			PINSEL1 &= ~(3 << 14);

			// X1 - ADC Ch1, X2 input with pull down
			FIO0DIR &= ~(1 << 24);
			FIO0DIR &= ~(1 << 22);
			PINSEL1 &= ~(3 << 16);
			PINSEL1 |= (1 << 16);
			PINMODE1 |= (3 << 12);
			AD0CR &= ~(3);
			AD0CR |= (1 << 1); // Ch1

			/*printf("X=%d Y=%d\n", ((X - (100 * SAMPLES * 2)) * 320)/((915 * SAMPLES * 2) - (100 * SAMPLES * 2)), 
					              ((Y - (125 * SAMPLES * 2)) * 240)/((900 * SAMPLES * 2) - (125 * SAMPLES * 2)));*/
			X /= SAMPLES;
			Y /= SAMPLES;

		
			X_temp = (X - 200) * 320 / 1600;
			Y_temp = (Y - 350) * 240 / 1350;

			if(X_temp < 0)
				X_temp = 0;
			if(X_temp > 320)
				X_temp = 320;

			if(Y_temp < 0)
				Y_temp = 0;
			if(Y_temp > 240)
				Y_temp = 240;

			printf("X=%d Y=%d\n", X_temp, Y_temp);
		}
	}

	return 0;
}
