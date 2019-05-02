// Lab6.c
// Runs on LM4F120 or TM4C123
// Use SysTick interrupts to implement a 3-key digital piano
// MOOC lab 13 or EE319K lab6 starter
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


#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "Sound.h"
#include "Piano.h"
#include "TExaS.h"


// Period =  80000000/32/Freq=2500000/Freq
#define C1   1194    // 2093 Hz
#define B1   1265    // 1975.5 Hz
#define BF1  1341    // 1864.7 Hz
#define A1   1420    // 1760 Hz
#define AF1  1505    // 1661.2 Hz
#define G1   1594    // 1568 Hz
#define GF1  1689    // 1480 Hz
#define F1   1790    // 1396.9 Hz
#define E1   1896    // 1318.5 Hz
#define EF1  2009    // 1244.5 Hz
#define D1   2128    // 1174.7 Hz
#define DF1  2255    // 1108.7 Hz
#define C    2389    // 1046.5 Hz
#define B    2531    // 987.8 Hz
#define BF   2681    // 932.3 Hz
#define A    2841    // 880 Hz
#define AF   3010    // 830.6 Hz
#define G    3189    // 784 Hz
#define GF   3378    // 740 Hz
#define F    3579    // 698.5 Hz
#define E    3792    // 659.3 Hz
#define EF   4018    // 622.3 Hz
#define D    4257    // 587.3 Hz
#define DF   4510    // 554.4 Hz
#define C0   4778    // 523.3 Hz
#define B0   5062    // 493.9 Hz
#define BF0  5363    // 466.2 Hz
#define A0   5682    // 440 Hz
#define AF0  6020    // 415.3 Hz
#define G0   6378    // 392 Hz
#define GF0  6757    // 370 Hz
#define F0   7159    // 349.2 Hz
#define E0   7584    // 329.6 Hz
#define EF0  8035    // 311.1 Hz
#define D0   8513    // 293.7 Hz
#define DF0  9019    // 277.2 Hz
#define C7   9556    // 261.6 Hz
#define B7   10124   // 246.9 Hz
#define BF7  10726   // 233.1 Hz
#define A7   11364   // 220 Hz
#define AF7  12039   // 207.7 Hz
#define G7   12755   // 196 Hz
#define GF7  13514   // 185 Hz
#define F7   14317   // 174.6 Hz
#define E7   15169   // 164.8 Hz
#define EF7  16071   // 155.6 Hz
#define D7   17026   // 146.8 Hz
#define DF7  18039   // 138.6 Hz
#define C6   19111   // 130.8 Hz

uint32_t IndianaJonesNotes [37] = {E,F,G,C,D,E,F,G,A,B,F1,A,B,C,D1,E1,E,F,G,C,D1,E1,F1,G,G,E1,D1,G,E1,D1,G,E1,D1,G,E1,D1,E};
uint32_t IndianaJonesTime [37] = {16/3,16,8,8/5,16/3,16,4/3,16/3,16,8,8/5,16/3,16,4,4,4,16/3,16,8,8/5,16/3,16,4/3,16/3,16,4,16/3,16,4,16/3,16,4,16/3,16,8,8,4};

// basic functions defined at end of startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void Song_Play(uint32_t NoteArr [], uint32_t TimeArr [], uint8_t length);

void Heartbeat_Song_Init (void) {
	SYSCTL_RCGCGPIO_R |= 0x20;
	__nop();
	__nop();
	GPIO_PORTF_DIR_R |= 0x04;
	GPIO_PORTF_DEN_R |= 0x15;
}

int main(void){      
  TExaS_Init(SW_PIN_PE3210,DAC_PIN_PB3210,ScopeOn);    // bus clock at 80 MHz
  Piano_Init();
  Sound_Init();
	Heartbeat_Song_Init();
  // other initialization
  EnableInterrupts();
  while(1){
		if (Piano_In() == 0x01) {
			Sound_Play(A); //Plays A Note if PE0 Pressed
		}
		if (Piano_In() == 0x02) {
			Sound_Play(B); //Plays B Note if PE1 Pressed
		}
		if (Piano_In() == 0x04) {
			Sound_Play(DF1); //Plays C Note if PE2 Pressed
		}
		if (Piano_In() == 0x00) {
			Sound_Play(0); //Plays Nothing if No/Multiple Buttons are Pressed
		}
		if ((GPIO_PORTF_DATA_R & 0x10) == 0x10) {
			while(1){
				if ((GPIO_PORTF_DATA_R & 0x10) == 0) {
					Song_Play(IndianaJonesNotes, IndianaJonesTime, 37);
					break;
				}
			}
		}
		GPIO_PORTF_DATA_R ^= 0x04; //Heatbeat Indictor
  }    
}
