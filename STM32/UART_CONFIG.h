// UART_CONFIG.h
#ifndef __UART_CONFIG_H
#define __UART_CONFIG_H

#include <stdint.h> 


#define GPIOA_BASE         ((uint32_t)0x40010800) // Ð?a ch? co s? cho GPIOA
#define USART1_BASE        ((uint32_t)0x40013800) // Ð?a ch? co s? cho USART1
#define RCC_BASE           ((uint32_t)0x40021000) // Ð?a ch? co s? cho RCC

#define RCC_APB2ENR  (*((volatile uint32_t *)(RCC_BASE + 0x18)))
#define GPIOA_CRH    (*((volatile uint32_t *)(GPIOA_BASE + 0x04)))
#define USART1_SR    (*((volatile uint32_t *)(USART1_BASE + 0x00)))
#define USART1_DR    (*((volatile uint32_t *)(USART1_BASE + 0x04)))
#define USART1_BRR   (*((volatile uint32_t *)(USART1_BASE + 0x08)))
#define USART1_CR1   (*((volatile uint32_t *)(USART1_BASE + 0x0C)))


// --- KHAI BÁO CÁC HÀM ---
void simple_delay(uint32_t count);
void USART1_SendChar(char data);
void USART1_SendString(char *str);
void USART1_Config(void);

#endif // __UART_CONFIG_H