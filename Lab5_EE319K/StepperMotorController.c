// StepperMotorController.c starter file EE319K Lab 5
// Runs on TM4C123
// Finite state machine to operate a stepper motor.  
// Jonathan Valvano
// January 18, 2019

// Hardware connections (External: two input buttons and four outputs to stepper motor)
//  PA5 is Wash input  (1 means pressed, 0 means not pressed)
//  PA4 is Wiper input  (1 means pressed, 0 means not pressed)
//  PE5 is Water pump output (toggle means washing)
//  PE4-0 are stepper motor outputs 
//  PF1 PF2 or PF3 control the LED on Launchpad used as a heartbeat
//  PB6 is LED output (1 activates external LED on protoboard)

#include "SysTick.h"
#include "TExaS.h"
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"

void EnableInterrupts(void);
// edit the following only if you need to move pins from PA4, PE3-0      
// logic analyzer on the real board
#define PA4       (*((volatile unsigned long *)0x40004040))
#define PE50      (*((volatile unsigned long *)0x400240FC))

struct MotorState {
		uint8_t output;
		uint16_t dwell; //ms
		uint8_t next[4];
};

struct LEDState {
		uint8_t outputLED;
		uint8_t nextLED[4];
};

struct MotorState Motor[20] = {
//   {Output, Dwell, {00, 01, 10, 11}}
	{/*M0*/  0x01, 10, { 0,  1,  1,  0}},
	{/*M1*/  0x02, 10, { 1,  2,  2,  1}},
	{/*M2*/  0x04, 10, { 2,  3,  3,  2}},
	{/*M3*/  0x08, 10, { 3,  4,  4,  3}},
	{/*M4*/  0x10, 10, { 4,  5,  5,  4}},
	{/*M5*/  0x01, 10, { 5,  6,  6,  5}},
	{/*M6*/  0x02, 10, { 6,  7,  7,  6}},
	{/*M7*/  0x04, 10, { 7,  8,  8,  7}},
	{/*M8*/  0x08, 10, { 8,  9,  9,  8}},
	{/*M9*/  0x10, 10, { 9, 10, 10,  9}},
	{/*M10*/ 0x08, 10, {10, 11, 11, 10}},
	{/*M11*/ 0x04, 10, {11, 12, 12, 11}},
	{/*M12*/ 0x02, 10, {12, 13, 13, 12}},
	{/*M13*/ 0x01, 10, {13, 14, 14, 13}},
	{/*M14*/ 0x10, 10, {14, 15, 15, 14}},
	{/*M15*/ 0x08, 10, {15, 16, 16, 15}},
	{/*M16*/ 0x04, 10, {16, 17, 17, 16}},
	{/*M17*/ 0x02, 10, {17, 18, 18, 17}},
	{/*M18*/ 0x01, 10, {18, 19, 19, 18}},
	{/*M19*/ 0x10, 10, {19,  0,  0, 19}},
};
	
struct LEDState LED[2] = {
// {Output, {00, 01, 10, 11}}
	{/*L0*/ 0x00, {0, 0, 1, 0}},
	{/*L1*/ 0x20, {0, 0, 0, 0}} 
};
	
uint8_t CS = 0, LEDCS = 0;
uint8_t input;

void SendDataToLogicAnalyzer(void){
  UART0_DR_R = 0x80|(PA4<<2)|PE50;
}

int main(void){ 
  TExaS_Init(&SendDataToLogicAnalyzer);    // activate logic analyzer and set system clock to 80 MHz
  SysTick_Init();   
// you initialize your system here
	SYSCTL_RCGCGPIO_R = 0x33; //sets clock
	SysTick_Wait(4);

	GPIO_PORTA_DIR_R = 0x00; //enables Port A pins
	GPIO_PORTA_DEN_R = 0x60;
	GPIO_PORTA_PUR_R = 0x60;
	
	GPIO_PORTE_DIR_R = 0x3F; //enables Port E pins
	GPIO_PORTE_DEN_R = 0x3F;
	GPIO_PORTE_PUR_R = 0x3F;
	
	GPIO_PORTF_DIR_R = 0x04; //enables Port F pins
	GPIO_PORTF_DEN_R = 0x04;
	GPIO_PORTF_PUR_R = 0x04;
	
/*	GPIO_PORTB_DIR_R = 0x20;
	GPIO_PORTB_DEN_R = 0x20;
	GPIO_PORTB_PUR_R = 0x20; */
	
  EnableInterrupts();
	
  while(1){
		GPIO_PORTE_DATA_R = LED[LEDCS].outputLED | Motor[CS].output; //Total Output = LED Output OR Motor Output (Combination of the Two)
// output
		SysTick_Wait1ms(Motor[CS].dwell);
// wait
		input = (GPIO_PORTA_DATA_R >> 4) & 0x03; //Designates Input
// input
		CS = Motor[CS].next[input];
		LEDCS = LED[LEDCS].nextLED[input];
// next
		GPIO_PORTF_DATA_R ^= 0x04; //Alternates Onboard LED to Indicate the Program is Still Running
  }
}


