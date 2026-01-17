#include "SYSCLK.h"
#include "stm32f10x.h" 

void SYSCLK_INIT(void) {
    //uint32_t timeout;

    //  HSE
    RCC_CR |= HSE_ON; // RCC_CR =  RCC_CR | HSE_ON
    //timeout = HSE_STARTUP_TIME;
		while ((READ_BIT1(RCC_CR, HSE_RDY)!=HSE_RDY)){
			__NOP();
		}
    if (RCC_CR & HSE_RDY) {
        //  HSE OK 
        FLASH_ACR |= FLASH_ACR_PRFTBE1; // set buffer
        FLASH_ACR &= ~FLASH_ACR_LATENCY1; // clear
        FLASH_ACR |= FLASH_ACR_LATENCY_21; // set 2 wait states

        // PLL = HSE * 9
        RCC_CFGR &= ~((0xF << 18) | (1 << 16));// clear
        RCC_CFGR |= (PLL_SRC_HSE | PLL_MUL_9);
    } else {
        // HSE- HSI/2 
        FLASH_ACR |= FLASH_ACR_PRFTBE;
        FLASH_ACR &= ~FLASH_ACR_LATENCY;
        FLASH_ACR |= FLASH_ACR_LATENCY_2;

         //PLL = HSI/2 * 9
        RCC_CFGR &= ~((0xF << 18) | (1 << 16));
        RCC_CFGR |= PLL_MUL_9;   
    }

    //  PLL
    RCC_CR |= PLL_ON;
    //timeout = PLL_STARTUP_TIME;
    while ((READ_BIT1(RCC_CR, PLL_RDY)!= PLL_RDY));
    if (!(RCC_CR & PLL_RDY)) {
        //  HSI 8 MHz
        return;
    }

    //   SYSCLK = PLL
    RCC_CFGR &= ~(0x3); //clear bit
    RCC_CFGR |= RCC_CFGR_SW_PLL1; // set PLL as source
    while ((RCC_CFGR & (0x3 << 2)) != RCC_CFGR_SWS_PLL1);

    //  Prescaler APB1 = /2 (max 36 MHz)
    RCC_CFGR &= ~(0x7 << 8); // clear bit
    RCC_CFGR |= APB1_PRESCALER_DIV2;// APB1 = HCLK/2
		RCC_CFGR &= ~(0x7 << 24);
		RCC_CFGR |= 0x05000000;
}