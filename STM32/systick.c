#include "systick.h"

static uint32_t toggle_timer = 0;
const uint32_t TOGGLE_INTERVAL = 500;
void LED_Toggle(void)
{
    // Kiem tra trang thai hien tai (ODR)
    if (GPIOC->ODR & (1 << 13))
    {
        // Neu dang ON (Bit 13 dang LOW), tat (Dat Bit 13 len HIGH)
        GPIOC->BRR = (1 << 13); // Dùng BRR d? Reset (LOW)
    }
    else
    {
        // Neu dang OFF (Bit 13 dang HIGH), bat (Dat Bit 13 xuong LOW)
        GPIOC->BSRR = (1 << 13); // S? d?ng thanh ghi BRR (Bit Reset Register)
    }
}


void SysTick_Init(uint32_t HCLK_Frequency)
{
    // Tan so ngat (Ticks per second) = 1000 (1ms)
    uint32_t ticks = HCLK_Frequency / 1000; 

    // 1. Tai gia tri reload (So luong xung truoc khi ngat)
    SysTick->LOAD = (uint32_t)(ticks - 1); 

    // 2. Clear gia tri hien tai
    SysTick->VAL = 0UL; 

    // 3. Cau hinh CSR (Control and Status Register)
    // Bit 2 (CLKSOURCE): Chon xung clock la HCLK (0: AHB/8, 1: HCLK)
    // Bit 1 (TICKINT): Cho phep ngat SysTick
    // Bit 0 (ENABLE): Kich hoat SysTick Counter
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | 
                    SysTick_CTRL_TICKINT_Msk   | 
                    SysTick_CTRL_ENABLE_Msk;
	
		// 4. C?u hình Uu tiên cho SysTick (Preempt Priority 1)
    // SysTick có IRQn là -1, du?c di?u khi?n qua SCB->SHPR[2] (byte 28-31)
    // Ð?t giá tr? 1 vào 4 bit MSB
    // 0x01UL << 4 = 0x10. (Vì ô uu tiên b?t d?u t? bit 4)
    SCB->SHPR[2] &= ~(0xFFUL << 24); // Xóa
    SCB->SHPR[2] |= (0x01UL << 28);  // Giá tr? 1 (Preempt=1) vào 4 bit MSB
	
}


void SysTick_Handler(void)
{
    ms_counter++; // Tang bien dem thoi gian
		toggle_timer++;
    if (toggle_timer >= TOGGLE_INTERVAL)
    {
        LED_Toggle(); // Chuy?n d?i tr?ng thái LED
        toggle_timer = 0; // Reset b? d?m
    }
}


uint32_t Get_Current_Time_MS(void)
{
    // Tra ve bien dem
    return ms_counter; 
}