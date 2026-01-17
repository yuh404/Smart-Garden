#include "HCSR04.h"

// ===== GLOBAL VARIABLES =====
volatile uint32_t IC_Val1 = 0;
volatile uint32_t IC_Val2 = 0;
volatile uint8_t  Is_First_Captured = 0;
volatile uint16_t Distance = 0;

// ===== SIMPLE DELAY (NOP) =====
static void delay_us(uint32_t us)
{
    while(us--)
    {
        for (volatile uint32_t i = 0; i < 8; i++)
            __asm("nop");
    }
}

// ===== GPIO INIT =====
static void GPIO_Init_All(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_IOPBEN;

    /* PA2 - TRIG (Output Push Pull) */
    GPIOA->CRL &= ~(0xF << (2 * 4));
    GPIOA->CRL |=  (0x3 << (2 * 4));   // 50MHz Output PP
    GPIOA->BSRR = (1 << (2 + 16));     // LOW

    /* PA0 - ECHO (Floating input) */
    GPIOA->CRL &= ~(0xF << (0 * 4));
    GPIOA->CRL |=  (0x4 << (0 * 4));

    /* PB0 - LED (Output Push Pull) */
    GPIOB->CRL &= ~(0xF << (0 * 4));
    GPIOB->CRL |=  (0x3 << (0 * 4));
    GPIOB->BSRR = (1 << 0); // LED OFF
}

// ===== TIMER2 INPUT CAPTURE INIT =====
static void TIM2_IC_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->CR1 = 0;
    TIM2->PSC = 71;          // 1 MHz (1us)
    TIM2->ARR = 0xFFFF;

    /* CC1 mapped to TI1 */
    TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM2->CCMR1 |=  TIM_CCMR1_CC1S_0;

    /* No filter (fast edge) */
    TIM2->CCMR1 &= ~(TIM_CCMR1_IC1F);

    /* Rising edge first */
    TIM2->CCER &= ~(TIM_CCER_CC1P);
    TIM2->CCER |=  TIM_CCER_CC1E;

    /* Enable interrupt */
    TIM2->DIER |= TIM_DIER_CC1IE;

    /* NVIC */
    NVIC_EnableIRQ(TIM2_IRQn);

    /* Start timer */
    TIM2->CR1 |= TIM_CR1_CEN;
}

// ===== PUBLIC INIT =====
void HCSR04_Init(void)
{
    GPIO_Init_All();
    TIM2_IC_Init();
}

// ===== TRIGGER =====
void HCSR04_Trigger(void)
{
    TIM2->CNT = 0;   // reset counter before measurement

    GPIOA->BSRR = (1 << (2 + 16)); // LOW
    delay_us(2);

    GPIOA->BSRR = (1 << 2);        // HIGH
    delay_us(12);                  // >=10us
    GPIOA->BSRR = (1 << (2 + 16)); // LOW
}

// ===== TIM2 IRQ HANDLER =====
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_CC1IF)
    {
        uint32_t capture = TIM2->CCR1;

        if (Is_First_Captured == 0)
        {
            IC_Val1 = capture;
            Is_First_Captured = 1;

            /* Rising -> Falling (disable before change) */
            TIM2->CCER &= ~TIM_CCER_CC1E;
            TIM2->CCER |=  TIM_CCER_CC1P;
            TIM2->CCER |=  TIM_CCER_CC1E;
        }
        else
        {
            IC_Val2 = capture;

            uint32_t diff;
            if (IC_Val2 >= IC_Val1)
                diff = IC_Val2 - IC_Val1;
            else
                diff = 0xFFFF - IC_Val1 + IC_Val2;

            Distance = diff / 58;   // cm

            /* LED logic */
           

            Is_First_Captured = 0;

            /* Falling -> Rising */
            TIM2->CCER &= ~TIM_CCER_CC1E;
            TIM2->CCER &= ~TIM_CCER_CC1P;
            TIM2->CCER |=  TIM_CCER_CC1E;
        }

        TIM2->SR &= ~TIM_SR_CC1IF;
    }
}
