#ifndef __NVIC_H
#define __NVIC_H
#include "stm32f10x.h"                  // Device header

void NVIC_Config_USART1(uint8_t PreemptPriority, uint8_t SubPriority);
void NVIC_Config_TIM2(uint8_t PreemptPriority, uint8_t SubPriority);

#endif