// Piano.c
// This software configures the off-board piano keys
// Lab 6 requires a minimum of 3 keys, but you could have more
// Runs on LM4F120 or TM4C123
// Program written by: Ryan Root and Abhishek Sridhar
// Date Created: 3/6/17 
// Last Modified: 3/24/19 
// Lab number: 6
// Hardware connections
// PE0 - PE2 are inputs corresponding to buttons
// PB0 outputs to Largest kohm resistor
// PB1 outputs to 1/2 Largest kohm resistor
// PB2 outputs to 1/4 Largest kohm resistor
// PB3 outputs to 1/8 Largest kohm resistor

// Code files contain the actual implemenation for public functions
// this file also contains an private functions and private data
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"

// **************Piano_Init*********************
// Initialize piano key inputs, called once to initialize the digital ports
// Input: none 
// Output: none
void Piano_Init(void){
	SYSCTL_RCGCGPIO_R |= 0x10;
	__nop();
	__nop();
	GPIO_PORTE_DIR_R |= 0x00;
	GPIO_PORTE_DEN_R |= 0x07;
}

// **************Piano_In*********************
// Input from piano key inputs 
// Input: none 
// Output: 0 to 7 depending on keys
//   0x01 is just Key0, 0x02 is just Key1, 0x04 is just Key2
//   bit n is set if key n is pressed
uint32_t Piano_In(void){
  if ((GPIO_PORTE_DATA_R & 0x07)  == 0x01) {
		return 0x01;
	}
	if ((GPIO_PORTE_DATA_R & 0x07)  == 0x02) {
		return 0x02;
	}
	if ((GPIO_PORTE_DATA_R & 0x07)  == 0x04) {
		return 0x04;
	}
	return 0x00;
}
