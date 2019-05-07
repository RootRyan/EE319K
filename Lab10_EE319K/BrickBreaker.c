// SpaceInvaders.c
// Runs on LM4F120/TM4C123
// Jonathan Valvano and Daniel Valvano
// This is a starter project for the EE319K Lab 10

// Last Modified: 11/20/2018 
// http://www.spaceinvaders.de/
// sounds at http://www.classicgaming.cc/classics/spaceinvaders/sounds.php
// http://www.classicgaming.cc/classics/spaceinvaders/playguide.php
/* This example accompanies the books
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2018

   "Embedded Systems: Introduction to Arm Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2018

 Copyright 2018 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
// ******* Possible Hardware I/O connections*******************
// Slide pot pin 1 connected to ground
// Slide pot pin 2 connected to PD2/AIN5
// Slide pot pin 3 connected to +3.3V 
// fire button connected to PE0
// special weapon fire button connected to PE1
// 8*R resistor DAC bit 0 on PB0 (least significant bit)
// 4*R resistor DAC bit 1 on PB1
// 2*R resistor DAC bit 2 on PB2
// 1*R resistor DAC bit 3 on PB3 (most significant bit)
// LED on PB4
// LED on PB5

// Backlight (pin 10) connected to +3.3 V
// MISO (pin 9) unconnected
// SCK (pin 8) connected to PA2 (SSI0Clk)
// MOSI (pin 7) connected to PA5 (SSI0Tx)
// TFT_CS (pin 6) connected to PA3 (SSI0Fss)
// CARD_CS (pin 5) unconnected
// Data/Command (pin 4) connected to PA6 (GPIO), high for data, low for command
// RESET (pin 3) connected to PA7 (GPIO)
// VCC (pin 2) connected to +3.3 V
// Gnd (pin 1) connected to ground

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "ST7735.h"
#include "Random.h"
#include "PLL.h"
#include "ADC.h"
#include "Images.h"
#include "Sound.h"
#include "Timer0.h"
#include "Timer1.h"

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void Delay100ms(uint32_t count); // time delay in 0.1 seconds
//void IntroScreen(void);

typedef enum {dead, alive} blockStat;
struct blockLoc{
	uint8_t x;
	uint8_t y;
	blockStat stat;
};

typedef struct blockLoc blockset;
blockset Block[6][5];
int32_t Score, Lives;
int32_t BlockWidth, BlockHeight, DeadCount;
int32_t BallX, BallY, BallSize;
int32_t BallXVel, BallYVel;
int32_t PaddleX, PaddleY, PaddleWidth, PaddleMax, PaddleChangeInd;
int32_t Speed = 1250000;
int32_t Indicator, ResetInd;

void PortEInit(void){
	SYSCTL_RCGCGPIO_R |= 0x10;
	__nop();
	__nop();
	GPIO_PORTE_DIR_R |= 0x03;	//For pins PE0,PE1
	GPIO_PORTE_DEN_R |= 0x03;	//For pins PE0,PE1
}

void RisingEdge_PortEInit(void){
	SYSCTL_RCGCGPIO_R |= 0x10;	//(a) activate clock for Port E
	
	__nop();
	__nop();
	__nop();
	__nop();
	
	GPIO_PORTE_AMSEL_R &= 0x03;		//(b) disable analog function on PE0, PE1
	GPIO_PORTE_PCTL_R &= ~0x000000FF; // configure PE0, PE1 as GPIO
	GPIO_PORTE_DIR_R &= ~0x03;		// Make PE0, PE1 Inputs	
	GPIO_PORTE_AFSEL_R &= ~0x03;  //     disable alt funct on PE0, PE1
  GPIO_PORTE_DEN_R |= 0x03;			//     enable digital I/O on PE0, PE1
	
	GPIO_PORTE_IS_R &= ~0x03;     // (c) PE0,PE1 is edge-sensitive
  GPIO_PORTE_IBE_R &= ~0x03;    //     PE0,PE1 is not both edges
  //GPIO_PORTE_IEV_R &= ~0x03;	//     PE0,PE1 falling edge event
	GPIO_PORTE_IEV_R |= 0x03;			//     PE0,PE1 rising edge event
  GPIO_PORTE_ICR_R = 0x03;      // (d) clear flag0, flag1
  GPIO_PORTE_IM_R |= 0x03;      // (e) enable interrupt on PE0,PE1
  NVIC_PRI1_R = (NVIC_PRI1_R&0xFFFFFF00)|0x00000040; // (f) priority 2
  NVIC_EN0_R = 0x00000010;      // (g) enable interrupt 4 in NVIC
  EnableInterrupts();           // (h) Clears the I bit
}

void SysTick_Init(void){
	NVIC_ST_CTRL_R = 0x00; //Disable SysTick During Init
	NVIC_ST_RELOAD_R = Speed;	//Variable to change
	NVIC_ST_CURRENT_R = 0x00;
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x00FFFFFF) | 0x60000000; //Sets Interrupt Priority to 3
	NVIC_ST_CTRL_R = 0x07; //Enable Interrupt	
}

void IntroScreen(void){
	SysTick_Init();
	ST7735_FillScreen(0x0000);            // set screen to black
	ST7735_SetCursor(0,0);
	ST7735_OutString("WELCOME TO");
	ST7735_SetCursor(0,1);
	ST7735_OutString("BRICK BREAKER");
	ST7735_SetCursor(0,4);
	ST7735_OutString("Press Any Button");
	ST7735_SetCursor(0,5);
	ST7735_OutString("To Start");
	ST7735_SetCursor(0,7);
	ST7735_OutString("Tip: You can change");
	ST7735_SetCursor(0,8);
	ST7735_OutString("your paddle size");
	ST7735_SetCursor(0,9);
	ST7735_OutString("once every 5 blocks");
	RisingEdge_PortEInit();
	//while(1);
}

void gameInit(void){
	ST7735_FillScreen(0x0000);
	
	Lives = 3;
	ST7735_SetCursor(0,0);
	ST7735_OutString("Lives: ");
	ST7735_SetCursor(7,0);
	LCD_OutDec(Lives);
	
	Score = 0;
	ST7735_SetCursor(9,0);
	ST7735_OutString("Score: ");
	ST7735_SetCursor(16,0);
	LCD_OutDec(Score);
	
	ST7735_FillRect(0,10,120,1,ST7735_YELLOW);
	
	BlockWidth = 20;
	BlockHeight = 10;
	DeadCount = 0;
	
	for(uint8_t row = 0; row < 6; row++){
		for(uint8_t col = 0; col < 5; col++){
			Block[row][col].stat = alive;
			Block[row][col].x = 20*row;
			Block[row][col].y = 10*col + 11;
			ST7735_FillRect(Block[row][col].x, Block[row][col].y, BlockWidth, BlockHeight, Random32());
		}
	}
	
	PaddleChangeInd = 0;
	PaddleWidth = 30;
	PaddleMax = 90;
	PaddleX = ((ADC_In()*PaddleMax)/4095);
	PaddleY = 140;
	ST7735_FillRect(PaddleX,PaddleY,PaddleWidth,5,0xFFFF);
	
	BallXVel = 1;
	BallYVel = -1;
	
	BallSize = 4;
	BallX = PaddleX + (PaddleWidth/2) - (BallSize/2);
	BallY = PaddleY - BallSize + BallYVel;
	ST7735_FillRect(BallX,BallY,BallSize,BallSize,0xFFFF);
	
	Indicator = 1;
}

void GameOver(void){
	DisableInterrupts();
	Indicator = 0;
	ST7735_FillScreen(0x0000);    
	ST7735_SetCursor(0,0);
	ST7735_OutString("GAME OVER--YOU LOSE");
	ST7735_SetCursor(0,1);
	ST7735_OutString("Score: ");
	LCD_OutDec(Score);
	ST7735_SetCursor(0,2);
	ST7735_OutString("Blocks Hit: ");
	LCD_OutDec(DeadCount);
	ST7735_SetCursor(0,4);
	ST7735_OutString("Press Any Button");
	ST7735_SetCursor(0,5);
	ST7735_OutString("To Return To Menu");
	GPIO_PORTE_IM_R ^= 0x03;      // Disable interrupt on PE0,PE1
	while(1){
		if((GPIO_PORTE_DATA_R == 0x02) || (GPIO_PORTE_DATA_R == 0x01)){
			ResetInd = 1;
			break;
		}
	}
	//gameInit();
	//IntroScreen();	
}

void GameWon(void){
	DisableInterrupts();
	Indicator = 0;
	ST7735_FillScreen(0x0000);    
	ST7735_SetCursor(0,0);
	ST7735_OutString("YOU WIN!! :)");
	ST7735_SetCursor(0,1);
	ST7735_OutString("Score: ");
	LCD_OutDec(Score);
	ST7735_SetCursor(0,2);
	ST7735_OutString("Blocks Hit: ");
	LCD_OutDec(DeadCount);
	ST7735_SetCursor(0,4);
	ST7735_OutString("Press Any Button");
	ST7735_SetCursor(0,5);
	ST7735_OutString("To Return To Menu");
	GPIO_PORTE_IM_R ^= 0x03;      // Disable interrupt on PE0,PE1
	while(1){
		if((GPIO_PORTE_DATA_R == 0x02) || (GPIO_PORTE_DATA_R == 0x01)){
			ResetInd = 1;
			break;
		}
	}
	//gameInit();
	//IntroScreen();
}

void displayInfo(void){
	ST7735_SetCursor(0,0);
	ST7735_OutString("Lives:             ");
	ST7735_SetCursor(7,0);
	LCD_OutDec(Lives);
	
	ST7735_SetCursor(9,0);
	ST7735_OutString("Score: ");
	ST7735_SetCursor(16,0);
	LCD_OutDec(Score);
}

void paddleMove(void){
	ST7735_FillRect(PaddleX,PaddleY,PaddleWidth,5,0x0000);
	PaddleX = ((ADC_In()*PaddleMax)/4095) + 1;
	ST7735_FillRect(PaddleX,PaddleY,PaddleWidth,5,0xFFFF);
}

void resetBall(void){
	ST7735_FillRect(BallX,BallY,BallSize,BallSize,0x0000);
	BallX = PaddleX + (PaddleWidth/2) + (BallSize/2);
	BallYVel = -1;
	BallY = PaddleY - BallSize + BallYVel;
	ST7735_FillRect(BallX,BallY,BallSize,BallSize,0xFFFF);
}

uint8_t checkCollision(void){
	if(BallY == 159-BallSize){
		return 0;
	}
	if((BallX == 119-BallSize) || (BallX == 0)){
		BallXVel *= -1;
	}
	if(BallY == 11){
		BallYVel *= -1;
	}
	if ((BallY == PaddleY - BallSize) && ((BallX > PaddleX - BallSize) || (BallX == PaddleX)) && (BallX < PaddleX + PaddleWidth + BallSize)){
		BallYVel *= -1;
		/*BallYVel = ((int32_t)(Random32()%3)+1)*-1; 
		if(BallXVel > 0){
			BallXVel = (int32_t)(Random32()%3)+1;
		} else {
			BallXVel = ((int32_t)(Random32()%3)+1)*-1;
		}*/
	}
	for(uint8_t row = 0; row < 6; row++){
		for(uint8_t col = 0; col < 5; col++){
			if(((BallY == Block[row][col].y + BlockHeight) || (BallY == Block[row][col].y - BallSize))  && (BallX > Block[row][col].x) && (BallX < Block[row][col].x + BlockWidth) && (Block[row][col].stat == alive)){
				Block[row][col].stat = dead;
				BallYVel *= -1;
				ST7735_FillRect(Block[row][col].x,Block[row][col].y,BlockWidth,BlockHeight,0x0000);
				Score += Lives*10;
				DeadCount++;
			}
			if(((BallX == Block[row][col].x - BallSize) || (BallX == Block[row][col].x + BlockWidth)) && (BallY >= Block[row][col].y) && (BallY <= Block[row][col].y + BlockHeight) && (Block[row][col].stat == alive)){
				Block[row][col].stat = dead;
				BallXVel *= -1;
				ST7735_FillRect(Block[row][col].x,Block[row][col].y,BlockWidth,BlockHeight,0x0000);
				Score += Lives*10;
				DeadCount++;
			}
		}
	}
	return 1;
}

void ballMove(void){
	uint8_t status;
	status = checkCollision();
	if(status == 1){
		if (DeadCount == 30){
		GameWon();
		}
		ST7735_FillRect(BallX,BallY,BallSize,BallSize,0x0000);
		BallX = BallX + BallXVel;
		BallY = BallY + BallYVel;
		ST7735_FillRect(BallX,BallY,BallSize,BallSize,0xFFFF);
	}
	if(status == 0){
		Lives--;
		if (Lives == 0){
			GameOver();
		} else {
			resetBall();
		}
	}
}

void SysTick_Handler(void){
	if(Indicator != 0){
		uint32_t currentScore = Score, currentLives = Lives;
		paddleMove();
		ballMove();
		if (((currentScore != Score) || (currentLives != Lives)) && (Indicator != 0)){
			displayInfo();
		}
	}
}

void GPIOPortE_Handler(void){
	if(Indicator != 0){
		if((GPIO_PORTE_RIS_R & 0x01) != 0){
			GPIO_PORTE_ICR_R = 0x01;
			//NVIC_ST_RELOAD_R ^= Speed; //alternates between no reload and regular reload
			//OR
			if((NVIC_ST_CTRL_R & 0x07) == 0x07){
				ST7735_SetCursor(0,0);
				ST7735_OutString("    Game Paused     ");
			} else {
				displayInfo();
			}
			NVIC_ST_CTRL_R ^= 0x07; //activates/deactivates SysTick
		}
		if((GPIO_PORTE_RIS_R & 0x02) != 0){
			GPIO_PORTE_ICR_R = 0x02;
			if(DeadCount >= PaddleChangeInd){
				ST7735_SetCursor(0,0);
				ST7735_OutString("Paddle Size Changed ");
				PaddleChangeInd += 5;
				ST7735_FillRect(PaddleX,PaddleY,PaddleWidth,5,0x0000);
				PaddleWidth = (((int32_t)(Random32()%4))+1)*15;
				PaddleMax = 120 - PaddleWidth;
				paddleMove();
			} else {
				ST7735_SetCursor(0,0);
				ST7735_OutString("No Powers Available ");
			}
		}
	} else {
		if((GPIO_PORTE_RIS_R & 0x02) != 0){
			GPIO_PORTE_ICR_R = 0x02;
		}
		if((GPIO_PORTE_RIS_R & 0x01) != 0){
			GPIO_PORTE_ICR_R = 0x01;
		}
		Random_Init(NVIC_ST_CURRENT_R);
		gameInit();
	}
}

int main(void){
	PLL_Init(Bus80MHz);
	Indicator = 0;
	ResetInd = 0;
	ADC_Init();
	Output_Init();
  IntroScreen();
	while(1){
		if(ResetInd != 0){
			PLL_Init(Bus80MHz);
			Indicator = 0;
			ResetInd = 0;
			ADC_Init();
			Output_Init();
			IntroScreen();
		}
	}
}

int mainTest(void){
  PLL_Init(Bus80MHz);       // Bus clock is 80 MHz 
  //Random_Init(1);

	ADC_Init();
	//SysTick_Init();
	//RisingEdge_PortEInit();

  Output_Init();
  IntroScreen();
	
	
	
	//gameInit();
	//SysTick_Init();
	//RisingEdge_PortEInit();
	
  while(1){
  }

}
