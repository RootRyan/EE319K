// dac.c
// This software configures DAC output
// Lab 6 requires a minimum of 4 bits for the DAC, but you could have 5 or 6 bits
// Runs on LM4F120 or TM4C123
// Program written by: Ryan Root
// Date Created: 3/6/17 
// Last Modified: 5/3/19 
// Lab number: 10
// Hardware connections
// PE0 - PE2 are inputs corresponding to buttons
// PB0 outputs to Largest kohm resistor
// PB1 outputs to 1/2 Largest kohm resistor
// PB2 outputs to 1/4 Largest kohm resistor
// PB3 outputs to 1/8 Largest kohm resistor

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
// Code files contain the actual implemenation for public functions
// this file also contains an private functions and private data

// **************DAC_Init*********************
// Initialize 4-bit DAC, called once 
// Input: none
// Output: none
void DAC_Init(void){
	SYSCTL_RCGCGPIO_R |= 0x02;
	__nop();
	__nop();
	GPIO_PORTB_DIR_R |= 0x0F;
	GPIO_PORTB_DEN_R |= 0x0F;
}

// **************DAC_Out*********************
// output to DAC
// Input: 4-bit data, 0 to 15 
// Input=n is converted to n*3.3V/15
// Output: none
void DAC_Out(uint32_t data){
	GPIO_PORTB_DATA_R = data;
}
