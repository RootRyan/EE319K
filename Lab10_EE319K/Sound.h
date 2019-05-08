// Sound.h
// Runs on TM4C123 or LM4F120
// Prototypes for basic functions to play sounds from the
// original Space Invaders.
// Jonathan Valvano
// November 17, 2014


void Sound_Init(void);
void Sound_Play(const unsigned char *pt, uint32_t count);

void Timer0_Handler(void);

void Sound_Stop(void);
void Sound_IntroMusic(void);
void Sound_Bounce(void);
void Sound_Win(void);
//void Sound_Lose(void);
//void Sound_PowerUp(void);
void Sound_Explosion(void);
