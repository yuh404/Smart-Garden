#ifndef __HCSR04_H
#define __HCSR04_H

#include "stm32f10x.h"

// ===== PIN MAP =====
// TRIG  -> PA2
// ECHO  -> PA0 (TIM2_CH1)
// LED   -> PB0 (Active Low)

// ===== PUBLIC API =====
void HCSR04_Init(void);
void HCSR04_Trigger(void);

extern volatile uint32_t IC_Val1;
extern volatile uint32_t IC_Val2;
extern volatile uint8_t  Is_First_Captured;
extern volatile uint16_t Distance;

#endif
