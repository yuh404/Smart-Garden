#ifndef __SYSCLOCK_H
#define __SYSCLOCK_H
typedef unsigned int uint32_t;

#define RCC_CR             	(*((volatile unsigned long *) 0x40021000))
#define RCC_CFGR           	(*((volatile unsigned long *) 0x40021004))
#define RCC_CIR            	(*((volatile unsigned long *) 0x40021008))
#define FLASH_ACR          	(*((volatile unsigned long *) 0x40022000))
#define RCC_APB2ENR        	(*((volatile unsigned long *) 0x40021018))
#define RCC_BDCR						(*((volatile unsigned long *) 0x40021020))
#define PWR_CR							(*((volatile unsigned long *) 0x40007000))
#define GPIOC_ODR						(*((volatile unsigned long *) 0x4001100C))
#define GPIOC_CRH						(*((volatile unsigned long *) 0x40011004))


// FLASH config
#define FLASH_ACR_PRFTBE1       (1UL << 4U)
#define FLASH_ACR_LATENCY1      (0x7UL)
#define FLASH_ACR_LATENCY_21    (0x2UL)

// RCC CR bits
#define HSE_ON                 (1UL << 16U)
#define HSE_RDY                (1UL << 17U)
#define PLL_ON                 (1UL << 24U)
#define PLL_RDY                (1UL << 25U)

// PLL config
#define PLL_MUL_9              (0x7UL << 18)    // *9
#define PLL_SRC_HSE            (1UL << 16)      // HSE as PLL source


// CFGR bits
#define RCC_CFGR_SW_PLL1        (0x2UL)
#define RCC_CFGR_SWS_PLL1       (0x2UL << 2)


// Prescaler
#define APB1_PRESCALER_DIV2    (0x4UL << 8)


//BDCR bits
#define LSE_ON									(0x1UL)
#define LSE_RDY									(0x2UL)
#define RTC_ON									(0x1UL << 15)
#define RTC_SRC									(0x1UL << 8)


//PWR bits
#define DBP											(0x1UL << 8)


// Timeout
#define HSE_STARTUP_TIME       0x5000
#define PLL_STARTUP_TIME       0x5000

//MCO
#define MCO_SYSCLK							0x04000000
#define MCO_PLL_DIV2 						0x05000000

// Macro
#define READ_BIT1(REG, BIT)    ((uint32_t)((REG) & (BIT)))

void SYSCLK_INIT(void);

#endif /* __SYSCLOCK_H */