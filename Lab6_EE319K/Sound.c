// Sound.c
// This module contains the SysTick ISR that plays sound
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
#include "dac.h"
#include "../inc/tm4c123gh6pm.h"

uint8_t ind = 0;
uint8_t i = 0;
const uint8_t sinWave[32] = {8,9,11,12,13,14,14,15,15,15,14,14,13,12,11,9,8,7,5,4,3,2,2,1,1,1,2,2,3,4,5,7}; 

// **************Sound_Init*********************
// Initialize digital outputs and SysTick timer
// Called once, with sound/interrupts initially off
// Input: none
// Output: none
void Sound_Init(void){
  DAC_Init();
	NVIC_ST_CTRL_R = 0x00; //Disable SysTick During Init
	NVIC_ST_RELOAD_R = 0x00;
	NVIC_ST_CURRENT_R = 0x00;
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x00FFFFFF) | 0x40000000; //Sets Interrupt Priority to 2
	NVIC_ST_CTRL_R = 0x07; //Enable Interrupt	
}

// **************Sound_Play*********************
// Start sound output, and set Systick interrupt period 
// Sound continues until Sound_Play called again
// This function returns right away and sound is produced using a periodic interrupt
// Input: interrupt period
//           Units of period to be determined by YOU
//           Maximum period to be determined by YOU
//           Minimum period to be determined by YOU
//         if period equals zero, disable sound output
// Output: none
void Sound_Play(uint32_t period){
	if (period == 0) {
		NVIC_ST_RELOAD_R = 0;
	} else {
		NVIC_ST_RELOAD_R = period - 1;
	}
}

//Interrupt Service Routine
void SysTick_Handler(void){
	DAC_Out(sinWave[ind]);
	ind = (ind + 1) % 32;
}

void Timer0A_Init(uint32_t period) {
  SYSCTL_RCGCTIMER_R |= 0x01;   // 0) activate TIMER0
	TIMER0_CTL_R = 0x00000000;    // 1) disable TIMER0A during setup
	TIMER0_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER0_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER0_TAILR_R = period - 1;  // 4) reload value
  TIMER0_TAPR_R = 0;            // 5) bus clock resolution
  TIMER0_ICR_R = 0x00000001;    // 6) clear TIMER0A timeout flag
  TIMER0_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x80000000; // 8) priority 4
// interrupts enabled in the main program after all devices initialized
// vector number 35, interrupt number 19
  NVIC_EN0_R = 1<<19;           // 9) enable IRQ 19 in NVIC
  TIMER0_CTL_R = 0x00000001;    // 10) enable TIMER0A
}

void Timer0A_Handler(void) {
	TIMER0_ICR_R = TIMER_ICR_TATOCINT;
	i++;
}

void Song_Play(uint32_t NoteArr [], uint32_t TimeArr [], uint8_t length) {
	uint8_t noteind = 50;
	i = 0;
	while (i < length) {
		if ((GPIO_PORTF_DATA_R & 0x01) == 0x00) {
			break;
		}
		if (noteind != i) {
			noteind = i;
			Timer0A_Init(80000000/TimeArr[i]);
			Sound_Play(NoteArr[i]);
		}
	}
	TIMER0_CTL_R = 0x00000000;
}
