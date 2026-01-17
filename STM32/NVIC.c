#include "NVIC.h"
void NVIC_Config_USART1(uint8_t PreemptPriority, uint8_t SubPriority)
{

    NVIC->ISER[USART1_IRQn >> 5] = (1 << (USART1_IRQn & 0x1F));


    uint32_t priority_value = (PreemptPriority << (4 - __NVIC_PRIO_BITS)) | 
                              (SubPriority << (4 - __NVIC_PRIO_BITS - 2)); 
                              
    // Dich chuyen sang ben trai 4 bit vi 4 bit uu tien nam o phan MSB cua moi byte
    priority_value <<= 4; 

    // Ghi vao thanh ghi IPR (Interrupt Priority Register)
    NVIC->IP[USART1_IRQn] = priority_value;

    // 3. Cau hinh Nhom Uu tien (Chon 2 bit Preempt va 2 bit Sub-Priority)
    // Thanh ghi SCB->AIRCR chi can set mot lan cho toan bo he thong
    // VECTKEY = 0x05FA de cho phep ghi
    // PRIGROUP = 0x05 (2 bit Preempt, 2 bit Sub-Priority)
    // SCB->AIRCR = 0x05FA0000 | (0x05 << 8); // Thong thuong da duoc set trong SystemInit()
}

void NVIC_Config_TIM2(uint8_t PreemptPriority, uint8_t SubPriority)
{
    // B?t ng?t trong ISER (TIM2_IRQn = 28)
    // TIM2_IRQn n?m trong ISER[0] (Index 0)
    NVIC->ISER[0] |= (1 << TIM2_IRQn); 

    // Tính toán giá tr? uu tiên
    uint32_t priority_value = (PreemptPriority << (4 - __NVIC_PRIO_BITS)) | 
                              (SubPriority << (4 - __NVIC_PRIO_BITS - 2)); 
                              
    // D?ch chuy?n sang bên trái 4 bit
    priority_value <<= 4; 

    // Ghi vào thanh ghi IPR (Interrupt Priority Register)
    NVIC->IP[TIM2_IRQn] = priority_value;
}