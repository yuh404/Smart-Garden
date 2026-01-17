#ifndef __systick_H
#define __systick_H
#include "stm32f10x.h"                  // Device header


static volatile uint32_t ms_counter = 0; 

// Ham ngat cua SysTick Timer (Can duoc dat ten chinh xac)
void SysTick_Handler(void);
void SysTick_Init(uint32_t HCLK_Frequency);
// Ham lay thoi gian phi chan (day la ham ban can)
uint32_t Get_Current_Time_MS(void);
void LED_Toggle(void);


#endif