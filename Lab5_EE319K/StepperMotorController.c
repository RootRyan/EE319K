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
	{/*M0*/  0x01, 50, { 0,  1,  1,  0}},
	{/*M1*/  0x02, 50, { 2,  2,  2,  2}},
	{/*M2*/  0x04, 50, { 3,  3,  3,  3}},
	{/*M3*/  0x08, 50, { 4,  4,  4,  4}},
	{/*M4*/  0x10, 50, { 5,  5,  5,  5}},
	{/*M5*/  0x01, 50, { 6,  6,  6,  6}},
	{/*M6*/  0x02, 50, { 7,  7,  7,  7}},
	{/*M7*/  0x04, 50, { 8,  8,  8,  8}},
	{/*M8*/  0x08, 50, { 9,  9,  9,  9}},
	{/*M9*/  0x10, 50, {10, 10, 10, 10}},
	{/*M10*/ 0x08, 50, {11, 11, 11, 11}},
	{/*M11*/ 0x04, 50, {12, 12, 12, 12}},
	{/*M12*/ 0x02, 50, {13, 13, 13, 13}},
	{/*M13*/ 0x01, 50, {14, 14, 14, 14}},
	{/*M14*/ 0x10, 50, {15, 15, 15, 15}},
	{/*M15*/ 0x08, 50, {16, 16, 16, 16}},
	{/*M16*/ 0x04, 50, {17, 17, 17, 17}},
	{/*M17*/ 0x02, 50, {18, 18, 18, 18}},
	{/*M18*/ 0x01, 50, {19, 19, 19, 19}},
	{/*M19*/ 0x10, 50, { 0,  0,  0,  0}},
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
	
	GPIO_PORTE_DIR_R = 0x3F; //enables Port E pins
	GPIO_PORTE_DEN_R = 0x3F;
	
	GPIO_PORTF_DIR_R = 0x04; //enables Port F pins
	GPIO_PORTF_DEN_R = 0x04;
	
/*GPIO_PORTB_DIR_R = 0x20;
	GPIO_PORTB_DEN_R = 0x20; */
	
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


