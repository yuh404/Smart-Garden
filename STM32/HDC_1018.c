//#include "HDC_1018.h"
//      // Ð? s? d?ng delayms()
//#include "stm32f10x_i2c.h" // Thu vi?n SPL cho I2C
//#include "stm32f10x_rcc.h"
//#include "stm32f10x_gpio.h"
//#include <stdint.h>
//#include <stdbool.h>

//// Th?i gian ch? t?i da cho các s? ki?n I2C (tính b?ng s? l?n l?p)
//#define I2C_TIMEOUT ((uint32_t)0x1000)
//#define DELAY_1MS_CONSTANT 3600
////
//// ===========================
////  INIT I2C1 (S? d?ng SPL)
//// ===========================
////

//void delayms(uint32_t ms)
//{
//    // Tính t?ng s? l?n l?p c?n thi?t
//    uint32_t total_iterations = ms * DELAY_1MS_CONSTANT;

//    // Vòng l?p r?ng
//    for (volatile uint32_t i = 0; i < total_iterations; i++)
//    {
//        // Ch?ng ng?n ch?n compiler t?i ?u hóa vòng l?p
//        __asm("nop"); 
//    }
//}



//void I2C1_Init(void)
//{
//    I2C_InitTypeDef I2C_InitStruct;
//    GPIO_InitTypeDef GPIO_InitStruct;

//    // 1. B?t Clock cho I2C1, GPIOB và AFIO
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

//    // 2. C?u hình GPIO (PB6-SCL, PB7-SDA)
//    // C? hai chân ph?i là Alternate Function Open Drain
//    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
//    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOB, &GPIO_InitStruct);

//    // 3. Reset I2C1 (Khuy?n ngh?)
//    I2C_DeInit(I2C1);
//    
//    // 4. C?u hình I2C Master
//    // Gi? d?nh PCLK1 = 36 MHz (t?n s? APB1)
//    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
//    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; // T?n su?t 50%
//    I2C_InitStruct.I2C_OwnAddress1 = 0x00;           // Ð?a ch? không dùng trong Master Mode
//    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
//    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//    // T?c d? 100 kHz (N?u PCLK1=36MHz, công th?c: CCR = 36000000 / (2 * 100000) = 180)
//    I2C_InitStruct.I2C_ClockSpeed = 100000; 

//    I2C_Init(I2C1, &I2C_InitStruct);

//    // B?t I2C1
//    I2C_Cmd(I2C1, ENABLE);
//}

////
//// ===========================
////  HÀM GIAO TI?P I2C (Dùng SPL Events)
//// ===========================
////

//// H? tr? ch? s? ki?n I2C
//bool I2C_WaitEvent(uint32_t I2C_Event)
//{
//    uint32_t timeout = I2C_TIMEOUT;
//    // Ch? s? ki?n ho?c h?t th?i gian ch?
//    while(I2C_CheckEvent(I2C1, I2C_Event) == RESET)
//    {
//        if((timeout--) == 0) return false; // L?i Timeout
//    }
//    return true; // Thành công
//}

///**
// * @brief Phát tín hi?u Start Condition và g?i d?a ch? thi?t b?
// * @param addr Ð?a ch? 8-bit (bao g?m R/W)
// * @param Direction I2C_Direction_Transmitter ho?c I2C_Direction_Receiver
// * @return true n?u thành công, false n?u th?t b?i (NACK/Timeout)
// */
////bool I2C_StartAndSendAddr(uint8_t addr, uint8_t Direction)
////{
////    // 1. Phát START Condition
////    I2C_GenerateSTART(I2C1, ENABLE);
////    if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return false; // Ch? c? SB

////    // 2. G?i d?a ch?
////    I2C_Send7bitAddress(I2C1, addr, Direction);

////    // 3. Ch? ACK t? Slave
////    if (Direction == I2C_Direction_Transmitter)
////    {
////        // Ch? ADDR và TX mode
////        return I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
////    }
////    else
////    {
////        // Ch? ADDR và RX mode
////        return I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
////    }
////}



//bool I2C_StartAndSendAddr(uint8_t addr, uint8_t Direction)
//{
//    // ***** FIX: CH? I2C BUS TR? V? TR?NG THÁI KHÔNG B?N (BUSY) *****
//    // B?t bu?c ch? BUSY flag clear tr??c khi phát START, th??ng là nguyên nhân gây l?i.
//    uint32_t timeout_busy = I2C_TIMEOUT;
//    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
//    {
//        if((timeout_busy--) == 0) return false; // L?i Timeout
//    }
//    // *******************************************************************
//    
//    // 1. Phát START Condition
//    I2C_GenerateSTART(I2C1, ENABLE);
//    // S? ki?n 0x0003: Ch? c? SB
//    if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return false; 

//    // 2. G?i d?a ch?
//    I2C_Send7bitAddress(I2C1, addr, Direction);

//    // 3. Ch? ACK t? Slave
//    if (Direction == I2C_Direction_Transmitter)
//    {
//        // S? ki?n 0x0007: Ch? ADDR và TX mode
//        return I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
//    }
//    else
//    {
//        // S? ki?n 0x0008: Ch? ADDR và RX mode
//        return I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
//    }
//}



////
//// ===========================
////  HÀM GIAO TI?P HDC1080 (Dùng SPL)
//// ===========================
////

//void HDC1080_Init(void)
//{
//    I2C_Cmd(I2C1, ENABLE); // Ð?m b?o I2C du?c b?t

//    // 1. Phát START và g?i d?a ch? GHI
//    if (!I2C_StartAndSendAddr(HDC1080_ADDR, I2C_Direction_Transmitter)) goto exit_error;

//    // 2. Ghi d?a ch? thanh ghi c?u hình (0x02)
//    I2C_SendData(I2C1, 0x02);
//    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING)) goto exit_error;

//    // 3. Ghi Configuration High Byte (0x10)
//    I2C_SendData(I2C1, 0x10); // 14-bit
//    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING)) goto exit_error;

//    // 4. Ghi Configuration Low Byte (0x00)
//    I2C_SendData(I2C1, 0x00);
//    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto exit_error; // Ch? BTF

//exit_error:
//    // Phát STOP Condition
//    I2C_GenerateSTOP(I2C1, ENABLE);
//    // N?u có l?i NACK/Timeout, hàm này s? tr? v? s?m
//}


////uint16_t HDC1080_ReadTemperatureRaw(void)
////{
////    uint8_t msb, lsb;
////    
////    // --- Giai do?n 1: G?i l?nh do (Pointer 0x00) ---
////    // B?t l?i ACK (N?u b? t?t ? l?n d?c tr?c)
////    I2C_AcknowledgeConfig(I2C1, ENABLE); 
////    if (!I2C_StartAndSendAddr(HDC1080_ADDR, I2C_Direction_Transmitter)) goto exit_temp_error;
////    
////    I2C_SendData(I2C1, 0x00); // Pointer: Nhi?t d?
////    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto exit_temp_error;
////    
////    I2C_GenerateSTOP(I2C1, ENABLE);
////    
////    // CH? TH?I GIAN CHUY?N Ð?I (t?ng lên 15ms)
////    delayms(15); 
////    
////    // --- Giai do?n 2: Ð?c d? li?u (Repeated START) ---
////    // Ph?i phát l?i START, SPL s? t? d?ng x? lý thành Repeated START n?u ch?a có STOP
////    I2C_GenerateSTART(I2C1, ENABLE); 
////    if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto exit_temp_error;
////    
////    I2C_Send7bitAddress(I2C1, HDC1080_ADDR, I2C_Direction_Receiver);
////    if (!I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) goto exit_temp_error;
////    
////    // *** S?A L?I QUAN TR?NG: Ð?c 2 bytes ***
////    
////    // Ð?c MSB (byte d?u tiên) v?i ACK (ACK v?n d?t ? trên)
////    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto exit_temp_error;
////    msb = I2C_ReceiveData(I2C1);

////    // Chu?n b? cho byte cu?i cùng: T?t ACK và phát STOP tr?c ti?p
////    // Quy t?c: T?t ACK TRU?C khi nh?n byte th? N-1 (MSB), phát STOP TRU?C khi d?c byte cu?i (LSB)
////    I2C_AcknowledgeConfig(I2C1, DISABLE); 
////    I2C_GenerateSTOP(I2C1, ENABLE); 
////    
////    // Ð?c LSB (byte cu?i cùng)
////    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto exit_temp_error;
////    lsb = I2C_ReceiveData(I2C1);
////    
////    return (msb << 8) | lsb;

////exit_temp_error:
////    // Ð?m b?o STOP lu?n du?c phát n?u x?y ra l?i
////    I2C_AcknowledgeConfig(I2C1, DISABLE);
////    I2C_GenerateSTOP(I2C1, ENABLE); 
////    return 0xFFFF;  
////}


////uint16_t HDC1080_ReadHumidityRaw(void)
////{
////    uint8_t msb, lsb;
////    
////    // --- Giai do?n 1: G?i l?nh do (Pointer 0x01) ---
////    // B?t l?i ACK
////    I2C_AcknowledgeConfig(I2C1, ENABLE); 
////    if (!I2C_StartAndSendAddr(HDC1080_ADDR, I2C_Direction_Transmitter)) goto exit_humi_error;
////    
////    I2C_SendData(I2C1, 0x01); // Pointer: Ð? ?m
////    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto exit_humi_error;
////    
////    I2C_GenerateSTOP(I2C1, ENABLE);
////    
////    // CH? TH?I GIAN CHUY?N Ð?I (t?ng lên 15ms)
////    delayms(15); 
////    
////    // --- Giai do?n 2: Ð?c d? li?u (Repeated START) ---
////    I2C_GenerateSTART(I2C1, ENABLE); 
////    if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto exit_humi_error;
////    
////    I2C_Send7bitAddress(I2C1, HDC1080_ADDR, I2C_Direction_Receiver);
////    if (!I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) goto exit_humi_error;

////    // Ð?c MSB (byte d?u tiên) v?i ACK (ACK v?n d?t ? trên)
////    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto exit_humi_error;
////    msb = I2C_ReceiveData(I2C1);

////    // Chu?n b? cho byte cu?i cùng: T?t ACK và phát STOP tr?c ti?p
////    I2C_AcknowledgeConfig(I2C1, DISABLE); 
////    I2C_GenerateSTOP(I2C1, ENABLE); 
////    
////    // Ð?c LSB (byte cu?i cùng)
////    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto exit_humi_error;
////    lsb = I2C_ReceiveData(I2C1);

////    return (msb << 8) | lsb;

////exit_humi_error:
////    // Ð?m b?o STOP lu?n du?c phát n?u x?y ra l?i
////    I2C_AcknowledgeConfig(I2C1, DISABLE);
////    I2C_GenerateSTOP(I2C1, ENABLE);
////    return 0xFFFF; 
////}

//// code khác

//void HDC1080_ReadTempHumiRaw(uint16_t *raw_temp, uint16_t *raw_humi)
//{
//    // M?c d?nh tr? v? l?i n?u quá trình I2C th?t b?i
//    *raw_temp = 0xFFFF;
//    *raw_humi = 0xFFFF;
//    
//    // Luôn b?t ACK d?m b?o I2C ho?t d?ng chính xác
//    I2C_AcknowledgeConfig(I2C1, ENABLE); 

//    // --- 1. G?i l?nh b?t d?u chuy?n d?i kép (Pointer 0x00) ---
//    if (!I2C_StartAndSendAddr(HDC1080_ADDR, I2C_Direction_Transmitter)) return; // G?i d?a ch? GHI
//    I2C_SendData(I2C1, 0x00); // Pointer: Nhi?t d?
//    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
//    I2C_GenerateSTOP(I2C1, ENABLE); // Phát STOP sau l?nh ghi

//    // --- 2. CH? TH?I GIAN CHUY?N Ð?I B?T BU?C ---
//    delayms(15); 

//    // --- 3. Ð?c liên ti?p 4 bytes (2T + 2H) ---
//    // Repeated Start
//    I2C_GenerateSTART(I2C1, ENABLE); 
//    if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return;
//    
//    // Chuy?n sang ch? d? Nh?n
//    I2C_Send7bitAddress(I2C1, HDC1080_ADDR, I2C_Direction_Receiver); // G?i d?a ch? Ð?C
//    if (!I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) return;
//    
//    // Byte 1 (T_MSB)
//    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
//    uint8_t temp_msb = I2C_ReceiveData(I2C1); 
//    
//    // Byte 2 (T_LSB)
//    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
//    uint8_t temp_lsb = I2C_ReceiveData(I2C1); 
//    
//    // ************ ÐI?M S?A L?I QUAN TR?NG ************
//    // Sau khi d?c byte N-2 (T_LSB), ph?i chu?n b? cho 2 byte cu?i (N-1 và N)
//    
//    // 1. T?t ACK (chu?n b? NACK cho byte cu?i H_LSB)
//    I2C_AcknowledgeConfig(I2C1, DISABLE); 
//    // 2. Kích ho?t Phát STOP (s? du?c g?i sau khi d?c byte N-1)
//    I2C_GenerateSTOP(I2C1, ENABLE); 
//    
//    // ************************************************
//    
//    // Byte 3 (H_MSB) - D?c byte N-1 (v?i ACK)
//    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
//    uint8_t humi_msb = I2C_ReceiveData(I2C1); 
//    
//    // Byte 4 (H_LSB) - D?c byte N (v?i NACK và STOP)
//    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
//    uint8_t humi_lsb = I2C_ReceiveData(I2C1);
//    
//    // T?ng h?p và luu k?t qu?
//    *raw_temp = (uint16_t)((temp_msb << 8) | temp_lsb);
//    *raw_humi = (uint16_t)((humi_msb << 8) | humi_lsb);
//    
//    // B?t l?i ACK d? các giao d?ch I2C ti?p theo ho?t d?ng bình thu?ng
//    I2C_AcknowledgeConfig(I2C1, ENABLE);
//}


//void I2C1_ClearBus(void)
//{
//    GPIO_InitTypeDef GPIO_InitStruct;
//    
//    // ********************************************
//    // GIAI DOAN 1: GIAI PHONG BUS VAT LY (GPIO TOGGLING)
//    // ********************************************
//    
//    // 1. T?t ngo?i vi I2C
//    I2C_Cmd(I2C1, DISABLE); 

//    // 2. C?u hình l?i SCL (PB6) và SDA (PB7) thành GPIO Output Open-Drain
//    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD; 
//    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOB, &GPIO_InitStruct);

//    // 3. Ð?m b?o SDA và SCL d?u HIGH (tr?ng thái r?nh)
//    GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);
//    delayms(1); 

//    // 4. Th?c hi?n 9 xung clock trên SCL d? reset bus
//    for (int i = 0; i < 9; i++) 
//    {
//        GPIO_ResetBits(GPIOB, GPIO_Pin_6); // SCL LOW
//        delayms(1); 
//        GPIO_SetBits(GPIOB, GPIO_Pin_6);   // SCL HIGH
//        delayms(1); 
//    }
//    
//    // 5. Th?c hi?n STOP Condition b?ng GPIO
//    GPIO_ResetBits(GPIOB, GPIO_Pin_7); // SDA LOW
//    delayms(1);
//    GPIO_SetBits(GPIOB, GPIO_Pin_7); // SDA HIGH
//    delayms(1); // Ch? STOP ?n d?nh
//    
//    // ********************************************
//    // GIAI DOAN 2: RESET NGOAI VI I2C BANG RCC
//    // ********************************************
//    
//    // 6. Reset I2C1 Peripheral bang RCC (Hard Reset)
//    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
//    // Vô hi?u hóa reset ngay l?p t?c
//    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE); 
//    
//    // ********************************************
//    // GIAI DOAN 3: KHOI TAO LAI NGOAI VI
//    // ********************************************
//    
//    // 7. G?i l?i hàm Init d? c?u hình l?i I2C sau khi Reset RCC
//    I2C1_Init(); 
//    
//    // Luu ý: Hàm I2C1_Init() s? t? c?u hình l?i GPIO v? AF_OD và g?i I2C_Cmd(ENABLE)
//}

//uint16_t HDC1080_ReadTemperatureRaw(void)
//{
//    uint8_t msb, lsb;
//    
//    I2C_AcknowledgeConfig(I2C1, ENABLE); 
//    
//    // --- Giai do?n 1: G?i l?nh do Nhi?t d? (Pointer 0x00) ---
//    if (!I2C_StartAndSendAddr(HDC1080_ADDR, I2C_Direction_Transmitter)) goto exit_temp_error;
//    
//    I2C_SendData(I2C1, 0x00); // Pointer: Nhi?t d?
//    // Ch? BTF d?m b?o byte dã du?c g?i hoàn toàn
//    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto exit_temp_error;
//    
//    I2C_GenerateSTOP(I2C1, ENABLE); 
//    
//    // CH? TH?I GIAN CHUY?N Ð?I
//    delayms(7); // 7ms là du cho 14-bit
//    
//    // --- Giai do?n 2: Ð?c d? li?u (Repeated START) ---
//    I2C_GenerateSTART(I2C1, ENABLE); 
//    if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto exit_temp_error;
//    
//    I2C_Send7bitAddress(I2C1, HDC1080_ADDR, I2C_Direction_Receiver); 
//    if (!I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) goto exit_temp_error;

//    // QUY T?C Ð?C 2 BYTE C?A SPL (Thao tác TRU?C khi d?c byte th? N-1)
//    // 1. T?t ACK (Chu?n b? NACK cho byte cu?i)
//    I2C_AcknowledgeConfig(I2C1, DISABLE); 
//    // 2. Phát STOP (S? du?c th?c hi?n sau khi d?c byte th? nh?t)
//    I2C_GenerateSTOP(I2C1, ENABLE); 
//    
//    // D?c MSB (Byte th? nh?t)
//    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto exit_temp_error;
//    msb = I2C_ReceiveData(I2C1);

//    // D?c LSB (Byte cu?i cùng) - S? ki?n BTF du?c set khi nh?n LSB và STOP du?c g?i
//    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto exit_temp_error;
//    lsb = I2C_ReceiveData(I2C1);
//    
//    return (uint16_t)(msb << 8) | lsb;

//exit_temp_error:
//    // D?m b?o STOP lu?n du?c phát (d? Bus kh?i b? k?t)
//    I2C_GenerateSTOP(I2C1, ENABLE); 
//    I2C_AcknowledgeConfig(I2C1, DISABLE); 
//    return 0xFFFF; // Báo l?i
//}

//uint16_t HDC1080_ReadHumidityRaw(void)
//{
//    uint8_t msb, lsb;
//    
//    I2C_AcknowledgeConfig(I2C1, ENABLE); 
//    
//    // --- Giai do?n 1: G?i l?nh do Ð? ?m (Pointer 0x01) ---
//    if (!I2C_StartAndSendAddr(HDC1080_ADDR, I2C_Direction_Transmitter)) goto exit_humi_error;
//    
//    I2C_SendData(I2C1, 0x01); // Pointer: Ð? ?m
//    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto exit_humi_error;
//    
//    I2C_GenerateSTOP(I2C1, ENABLE); 
//    
//    // CH? TH?I GIAN CHUY?N Ð?I
//    delayms(7); 
//    
//    // --- Giai do?n 2: Ð?c d? li?u (Repeated START) --
//    I2C_GenerateSTART(I2C1, ENABLE); 
//    if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto exit_humi_error;
//    
//    I2C_Send7bitAddress(I2C1, HDC1080_ADDR, I2C_Direction_Receiver); 
//    if (!I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) goto exit_humi_error;

//    // QUY T?C Ð?C 2 BYTE C?A SPL
//    I2C_AcknowledgeConfig(I2C1, DISABLE); 
//    I2C_GenerateSTOP(I2C1, ENABLE); 
//    
//    // D?c MSB
//    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto exit_humi_error;
//    msb = I2C_ReceiveData(I2C1);

//    // D?c LSB
//    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto exit_humi_error;
//    lsb = I2C_ReceiveData(I2C1);

//    return (uint16_t)(msb << 8) | lsb;

//exit_humi_error:
//    I2C_GenerateSTOP(I2C1, ENABLE);
//    I2C_AcknowledgeConfig(I2C1, DISABLE);
//    return 0xFFFF; 
//}




#include "HDC_1018.h"
    // Ð? s? d?ng delayms()
#include "stm32f10x_i2c.h" // Thu vi?n SPL cho I2C
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include <stdint.h>
#include <stdbool.h>

// Th?i gian ch? t?i da cho các s? ki?n I2C (tính b?ng s? l?n l?p)
#define I2C_TIMEOUT ((uint32_t)0x1000)
#define DELAY_1MS_CONSTANT 3600
//
// ===========================
//  INIT I2C1 (S? d?ng SPL)
// ===========================
//


void delay_ms(uint32_t ms)
{
    // Tính t?ng s? l?n l?p c?n thi?t
    uint32_t total_iterations = ms * DELAY_1MS_CONSTANT;

    // Vòng l?p r?ng
    for (volatile uint32_t i = 0; i < total_iterations; i++)
    {
        // Ch?ng ng?n ch?n compiler t?i ?u hóa vòng l?p
        __asm("nop"); 
    }
}



void I2C1_Init(void)
{
    I2C_InitTypeDef I2C_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

    // 1. B?t Clock cho I2C1, GPIOB và AFIO
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    // 2. C?u hình GPIO (PB6-SCL, PB7-SDA)
    // C? hai chân ph?i là Alternate Function Open Drain
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. Reset I2C1 (Khuy?n ngh?)
    I2C_DeInit(I2C1);
    
    // 4. C?u hình I2C Master
    // Gi? d?nh PCLK1 = 36 MHz (t?n s? APB1)
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; // T?n su?t 50%
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;           // Ð?a ch? không dùng trong Master Mode
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    // T?c d? 100 kHz (N?u PCLK1=36MHz, công th?c: CCR = 36000000 / (2 * 100000) = 180)
    I2C_InitStruct.I2C_ClockSpeed = 100000; 

    I2C_Init(I2C1, &I2C_InitStruct);

    // B?t I2C1
    I2C_Cmd(I2C1, ENABLE);
}

//
// ===========================
//  HÀM GIAO TI?P I2C (Dùng SPL Events)
// ===========================
//

// H? tr? ch? s? ki?n I2C
bool I2C_WaitEvent(uint32_t I2C_Event)
{
    uint32_t timeout = I2C_TIMEOUT;
    // Ch? s? ki?n ho?c h?t th?i gian ch?
    while(I2C_CheckEvent(I2C1, I2C_Event) == RESET)
    {
        if((timeout--) == 0) return false; // L?i Timeout
    }
    return true; // Thành công
}

/**
 * @brief Phát tín hi?u Start Condition và g?i d?a ch? thi?t b?
 * @param addr Ð?a ch? 8-bit (bao g?m R/W)
 * @param Direction I2C_Direction_Transmitter ho?c I2C_Direction_Receiver
 * @return true n?u thành công, false n?u th?t b?i (NACK/Timeout)
 */
bool I2C_StartAndSendAddr(uint8_t addr, uint8_t Direction)
{
    // 1. Phát START Condition
    I2C_GenerateSTART(I2C1, ENABLE);
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return false; // Ch? c? SB

    // 2. G?i d?a ch?
    I2C_Send7bitAddress(I2C1, addr, Direction);

    // 3. Ch? ACK t? Slave
    if (Direction == I2C_Direction_Transmitter)
    {
        // Ch? ADDR và TX mode
        return I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    }
    else
    {
        // Ch? ADDR và RX mode
        return I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
    }
}

//
// ===========================
//  HÀM GIAO TI?P HDC1080 (Dùng SPL)
// ===========================
//

void HDC1080_Init(void)
{
    I2C_Cmd(I2C1, ENABLE); // Ð?m b?o I2C du?c b?t

    // 1. Phát START và g?i d?a ch? GHI
    if (!I2C_StartAndSendAddr(HDC1080_ADDR, I2C_Direction_Transmitter)) goto exit_error;

    // 2. Ghi d?a ch? thanh ghi c?u hình (0x02)
    I2C_SendData(I2C1, 0x02);
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING)) goto exit_error;

    // 3. Ghi Configuration High Byte (0x10)
    I2C_SendData(I2C1, 0x10); // 14-bit
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING)) goto exit_error;

    // 4. Ghi Configuration Low Byte (0x00)
    I2C_SendData(I2C1, 0x00);
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto exit_error; // Ch? BTF

exit_error:
    // Phát STOP Condition
    I2C_GenerateSTOP(I2C1, ENABLE);
    // N?u có l?i NACK/Timeout, hàm này s? tr? v? s?m
}


uint16_t HDC1080_ReadTemperatureRaw(void)
{
    uint8_t msb, lsb;
    
    // --- Giai do?n 1: G?i l?nh do (Pointer 0x00) ---
    // B?t l?i ACK (N?u b? t?t ? l?n d?c tr?c)
    I2C_AcknowledgeConfig(I2C1, ENABLE); 
    if (!I2C_StartAndSendAddr(HDC1080_ADDR, I2C_Direction_Transmitter)) goto exit_temp_error;
    
    I2C_SendData(I2C1, 0x00); // Pointer: Nhi?t d?
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto exit_temp_error;
    
    I2C_GenerateSTOP(I2C1, ENABLE);
    
    // CH? TH?I GIAN CHUY?N Ð?I (t?ng lên 15ms)
    delay_ms(25); 
    
    // --- Giai do?n 2: Ð?c d? li?u (Repeated START) ---
    // Ph?i phát l?i START, SPL s? t? d?ng x? lý thành Repeated START n?u ch?a có STOP
    I2C_GenerateSTART(I2C1, ENABLE); 
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto exit_temp_error;
    
    I2C_Send7bitAddress(I2C1, HDC1080_ADDR, I2C_Direction_Receiver);
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) goto exit_temp_error;
    
    // *** S?A L?I QUAN TR?NG: Ð?c 2 bytes ***
    
    // Ð?c MSB (byte d?u tiên) v?i ACK (ACK v?n d?t ? trên)
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto exit_temp_error;
    msb = I2C_ReceiveData(I2C1);

    // Chu?n b? cho byte cu?i cùng: T?t ACK và phát STOP tr?c ti?p
    // Quy t?c: T?t ACK TRU?C khi nh?n byte th? N-1 (MSB), phát STOP TRU?C khi d?c byte cu?i (LSB)
    I2C_AcknowledgeConfig(I2C1, DISABLE); 
    I2C_GenerateSTOP(I2C1, ENABLE); 
    
    // Ð?c LSB (byte cu?i cùng)
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto exit_temp_error;
    lsb = I2C_ReceiveData(I2C1);
    
    return (msb << 8) | lsb;

exit_temp_error:
    // Ð?m b?o STOP lu?n du?c phát n?u x?y ra l?i
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    I2C_GenerateSTOP(I2C1, ENABLE); 
    return 0xFFFF;  
}


uint16_t HDC1080_ReadHumidityRaw(void)
{
    uint8_t msb, lsb;
    
    // --- Giai do?n 1: G?i l?nh do (Pointer 0x01) ---
    // B?t l?i ACK
    I2C_AcknowledgeConfig(I2C1, ENABLE); 
    if (!I2C_StartAndSendAddr(HDC1080_ADDR, I2C_Direction_Transmitter)) goto exit_humi_error;
    
    I2C_SendData(I2C1, 0x01); // Pointer: Ð? ?m
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto exit_humi_error;
    
    I2C_GenerateSTOP(I2C1, ENABLE);
    
    // CH? TH?I GIAN CHUY?N Ð?I (t?ng lên 15ms)
    delay_ms(15); 
    
    // --- Giai do?n 2: Ð?c d? li?u (Repeated START) ---
    I2C_GenerateSTART(I2C1, ENABLE); 
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto exit_humi_error;
    
    I2C_Send7bitAddress(I2C1, HDC1080_ADDR, I2C_Direction_Receiver);
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) goto exit_humi_error;

    // Ð?c MSB (byte d?u tiên) v?i ACK (ACK v?n d?t ? trên)
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto exit_humi_error;
    msb = I2C_ReceiveData(I2C1);

    // Chu?n b? cho byte cu?i cùng: T?t ACK và phát STOP tr?c ti?p
    I2C_AcknowledgeConfig(I2C1, DISABLE); 
    I2C_GenerateSTOP(I2C1, ENABLE); 
    
    // Ð?c LSB (byte cu?i cùng)
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto exit_humi_error;
    lsb = I2C_ReceiveData(I2C1);

    return (msb << 8) | lsb;

exit_humi_error:
    // Ð?m b?o STOP lu?n du?c phát n?u x?y ra l?i
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    I2C_GenerateSTOP(I2C1, ENABLE);
    return 0xFFFF; 
}

void HDC1080_ReadTempHumiRaw(uint16_t *raw_temp, uint16_t *raw_humi)
{
    // M?c d?nh tr? v? l?i n?u quá trình I2C th?t b?i
    *raw_temp = 0xFFFF;
    *raw_humi = 0xFFFF;
    
    // Luôn b?t ACK d?m b?o I2C ho?t d?ng chính xác
    I2C_AcknowledgeConfig(I2C1, ENABLE); 

    // --- 1. G?i l?nh b?t d?u chuy?n d?i kép (Pointer 0x00) ---
    if (!I2C_StartAndSendAddr(HDC1080_ADDR, I2C_Direction_Transmitter)) return; // G?i d?a ch? GHI
    I2C_SendData(I2C1, 0x00); // Pointer: Nhi?t d?
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
    I2C_GenerateSTOP(I2C1, ENABLE); // Phát STOP sau l?nh ghi

    // --- 2. CH? TH?I GIAN CHUY?N Ð?I B?T BU?C ---
    delay_ms(15); 

    // --- 3. Ð?c liên ti?p 4 bytes (2T + 2H) ---
    // Repeated Start
    I2C_GenerateSTART(I2C1, ENABLE); 
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return;
    
    // Chuy?n sang ch? d? Nh?n
    I2C_Send7bitAddress(I2C1, HDC1080_ADDR, I2C_Direction_Receiver); // G?i d?a ch? Ð?C
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) return;
    
    // Byte 1 (T_MSB)
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    uint8_t temp_msb = I2C_ReceiveData(I2C1); 
    
    // Byte 2 (T_LSB)
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    uint8_t temp_lsb = I2C_ReceiveData(I2C1); 
    
    // ************ ÐI?M S?A L?I QUAN TR?NG ************
    // Sau khi d?c byte N-2 (T_LSB), ph?i chu?n b? cho 2 byte cu?i (N-1 và N)
    
    // 1. T?t ACK (chu?n b? NACK cho byte cu?i H_LSB)
    I2C_AcknowledgeConfig(I2C1, DISABLE); 
    // 2. Kích ho?t Phát STOP (s? du?c g?i sau khi d?c byte N-1)
    I2C_GenerateSTOP(I2C1, ENABLE); 
    
    // ************************************************
    
    // Byte 3 (H_MSB) - D?c byte N-1 (v?i ACK)
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    uint8_t humi_msb = I2C_ReceiveData(I2C1); 
    
    // Byte 4 (H_LSB) - D?c byte N (v?i NACK và STOP)
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    uint8_t humi_lsb = I2C_ReceiveData(I2C1);
    
    // T?ng h?p và luu k?t qu?
    *raw_temp = (uint16_t)((temp_msb << 8) | temp_lsb);
    *raw_humi = (uint16_t)((humi_msb << 8) | humi_lsb);
    
    // B?t l?i ACK d? các giao d?ch I2C ti?p theo ho?t d?ng bình thu?ng
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}