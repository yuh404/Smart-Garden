#ifndef __DIENDUNG_H
#define __DIENDUNG_H
#include "stm32f10x.h"                  // Device header
#define ADC_CR2_EXTSEL_SWSTART        ((uint32_t)0x000E0000)
void ADC1_Register_Init_Calibrated(void);
uint16_t ADC1_Read_Channel(uint8_t channel);



#endif /* __DIENDUNG_H */