// UART_CONFIG.c
#include "UART_CONFIG.h"
#include <string.h> // C?n thi?t cho strlen/x? lý chu?i

// --- Ð?NH NGHIA (THÂN HÀM) CÁC HÀM ---

// Hàm Delay don gi?n
void simple_delay(uint32_t count)
{
    for (uint32_t i = 0; i < count; i++);
}

// Hàm g?i 1 ký t?
void USART1_SendChar(char data)
{
    // Ch? cho c? TXE (Transmit data register empty) du?c set (bit 7)
    while (!(USART1_SR & (1 << 7))); 
    
    // Ð?t byte d? li?u vào thanh ghi DR (Data Register) d? g?i
    USART1_DR = data;
}

// Hàm g?i chu?i
void USART1_SendString(char *str)
{
    while (*str != '\0') {
        USART1_SendChar(*str++);
    }
}

// Hàm c?u hình
void USART1_Config(void)
{
    // 1. Kích ho?t Xung Clock
    // B?t xung GPIOA (bit 2) và USART1 (bit 14) trên APB2ENR
    RCC_APB2ENR |= (1 << 2) | (1 << 14);

    // 2. C?u hình GPIOA.9 (TX) và GPIOA.10 (RX)
    // Xóa c?u hình cu cho PA9 (bits 4-7) và PA10 (bits 8-11)
    GPIOA_CRH &= ~((0xF << 4) | (0xF << 8)); 
    // PA9 (TX): MODE=11 (50MHz), CNF=10 (AF Push-Pull) -> 1011b = 0xB
    GPIOA_CRH |= (0xB << 4);
    // PA10 (RX): MODE=00 (Input), CNF=01 (Input Floating) -> 0100b = 0x4
    GPIOA_CRH |= (0x4 << 8);

    USART1_BRR = 0x271; 
    
    // B?t Transmitter (TE - bit 3)
    USART1_CR1 |= (1 << 3);
	 // b?t RE
		USART1_CR1 |= (1 << 2);
		
		// B?T NG?T
    USART1_CR1 |= (1 << 5);
    // Kích ho?t USART (UE - bit 13)
    USART1_CR1 |= (1 << 13);
}