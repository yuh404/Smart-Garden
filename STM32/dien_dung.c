#include "stm32f10x.h"   
#include "dien_dung.h"  // Device header
void ADC1_Register_Init_Calibrated(void)
{
    // Clock ADC 12 MHz (72/6)
    RCC->CFGR &= ~RCC_CFGR_ADCPRE;
    RCC->CFGR |=  RCC_CFGR_ADCPRE_DIV6;

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;


    // PA1 & PA3 analog mode
    GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
    GPIOA->CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3);

    // Sample time 55.5 cycles
    ADC1->SMPR2 |= (4 << 3);   // SMP1
    ADC1->SMPR2 |= (4 << 9);   // SMP3

//    // Scan mode
//    ADC1->CR1 |= ADC_CR1_SCAN;

//    // Sequence length 2 channels
//	
//    ADC1->SQR1 &= ~ADC_SQR1_L;
//    ADC1->SQR1 |= (1 << 20);

//    // SQ1 = channel 1, SQ2 = channel 3
//		ADC1->SQR3  = 0;
//    ADC1->SQR3 = (1 << 0) | (3 << 5);

    // External trigger SWSTART
    ADC1->CR2 &= ~ADC_CR2_EXTSEL;
    ADC1->CR2 |= ADC_CR2_EXTSEL_SWSTART;
    ADC1->CR2 |= ADC_CR2_EXTTRIG;

    // Turn on ADC
    ADC1->CR2 |= ADC_CR2_ADON;
    for (volatile int i = 0; i < 1000; i++);

    // Calibration
    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL);

    ADC1->CR2 |= ADC_CR2_ADON;
}

uint16_t ADC1_Read_Channel(uint8_t channel)
{
    uint32_t timeout = 100000;

    // Ch?n kênh (SQR3 ch? có 1 conversion)
    ADC1->SQR3 = channel;

    // Start conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Wait EOC
    while (!(ADC1->SR & ADC_SR_EOC))
    {
        if (--timeout == 0) return 0xFFFF;  // l?i
    }

    return ADC1->DR;  // giá tr? ADC
}
