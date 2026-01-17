//#ifndef __HDC_1018_H
//#define __HDC_1018_H

//#include "stm32f10x.h" // Thu vi?n cho các thanh ghi/struct c?a STM32
//#include <stdint.h>      // Cho các ki?u d? li?u uint8_t, uint16_t
//#include <stdbool.h>     // Cho ki?u d? li?u bool

//// ===================================
//// 1. Ð?NH NGHIA H?NG S? C?M BI?M
//// ===================================

//// Ð?a ch? 7-bit c?a HDC1080 là 0x40.
//// D?ch trái 1 bit d? t?o ra d?a ch? 8-bit.
//#define HDC1080_ADDR   (0x40 << 1) 

//// ===================================
//// 2. KHAI BÁO HÀM I2C C?P TH?P (Dùng SPL)
//// ===================================

///**
// * @brief Kh?i t?o ngo?i vi I2C1 và c?u hình GPIO (PB6/PB7) b?ng SPL.
// */
//void I2C1_Init(void);

///**
// * @brief H? tr? ch? các s? ki?n I2C.
// * @param I2C_Event S? ki?n c?n ch?.
// * @return true n?u s? ki?n x?y ra, false n?u Timeout.
// */
//bool I2C_WaitEvent(uint32_t I2C_Event);

///**
// * @brief Phát tín hi?u Start và g?i d?a ch? thi?t b? b?ng SPL.
// * @param addr Ð?a ch? 8-bit (dã d?ch trái).
// * @param Direction I2C_Direction_Transmitter ho?c I2C_Direction_Receiver.
// * @return true n?u thành công, false n?u th?t b?i (NACK/Timeout).
// */
//bool I2C_StartAndSendAddr(uint8_t addr, uint8_t Direction);

//// ===================================
//// 3. KHAI BÁO HÀM HDC1080
//// ===================================

///**
// * @brief Kh?i t?o c?m bi?n HDC1080 (Ghi thanh ghi c?u hình 14-bit).
// */
//void HDC1080_Init(void);

///**
// * @brief Th?c hi?n do và d?c giá tr? RAW c?a Nhi?t d?.
// * @return Giá tr? RAW 16-bit c?a nhi?t d? (0xFFFF n?u l?i).
// */
//uint16_t HDC1080_ReadTemperatureRaw(void);

///**
// * @brief Th?c hi?n do và d?c giá tr? RAW c?a Ð? ?m.
// * @return Giá tr? RAW 16-bit c?a d? ?m (0xFFFF n?u l?i).
// */
//uint16_t HDC1080_ReadHumidityRaw(void);

//// ===================================
//// 4. KHAI BÁO HÀM CHUY?N Ð?I 
//// ===================================

///**
// * @brief Chuy?n d?i giá tr? RAW c?a nhi?t d? sang d? Celsius (C).
// * @param raw_temp Giá tr? 16-bit d?c du?c t? c?m bi?n.
// * @return Nhi?t d? th?c t? ? d?ng float.
// */
//float HDC1080_ConvertTemperature(uint16_t raw_temp);

///**
// * @brief Chuy?n d?i giá tr? RAW c?a d? ?m sang d? ?m tuong d?i (%).
// * @param raw_humi Giá tr? 16-bit d?c du?c t? c?m bi?n.
// * @return Ð? ?m tuong d?i th?c t? ? d?ng float.
// */
//float HDC1080_ConvertHumidity(uint16_t raw_humi);
//void delayms(uint32_t ms);
//void HDC1080_ReadTempHumiRaw(uint16_t *raw_temp, uint16_t *raw_humi);
//void I2C1_ClearBus(void);
//#endif // __HDC_1018_H



#ifndef __HDC_1018_H
#define __HDC_1018_H

#include "stm32f10x.h" // Thu vi?n cho các thanh ghi/struct c?a STM32
#include <stdint.h>      // Cho các ki?u d? li?u uint8_t, uint16_t
#include <stdbool.h>     // Cho ki?u d? li?u bool

// ===================================
// 1. Ð?NH NGHIA H?NG S? C?M BI?M
// ===================================

// Ð?a ch? 7-bit c?a HDC1080 là 0x40.
// D?ch trái 1 bit d? t?o ra d?a ch? 8-bit.
#define HDC1080_ADDR   (0x40 << 1) 

// ===================================
// 2. KHAI BÁO HÀM I2C C?P TH?P (Dùng SPL)
// ===================================

/**
 * @brief Kh?i t?o ngo?i vi I2C1 và c?u hình GPIO (PB6/PB7) b?ng SPL.
 */
void I2C1_Init(void);

/**
 * @brief H? tr? ch? các s? ki?n I2C.
 * @param I2C_Event S? ki?n c?n ch?.
 * @return true n?u s? ki?n x?y ra, false n?u Timeout.
 */
bool I2C_WaitEvent(uint32_t I2C_Event);

/**
 * @brief Phát tín hi?u Start và g?i d?a ch? thi?t b? b?ng SPL.
 * @param addr Ð?a ch? 8-bit (dã d?ch trái).
 * @param Direction I2C_Direction_Transmitter ho?c I2C_Direction_Receiver.
 * @return true n?u thành công, false n?u th?t b?i (NACK/Timeout).
 */
bool I2C_StartAndSendAddr(uint8_t addr, uint8_t Direction);

// ===================================
// 3. KHAI BÁO HÀM HDC1080
// ===================================

/**
 * @brief Kh?i t?o c?m bi?n HDC1080 (Ghi thanh ghi c?u hình 14-bit).
 */
void HDC1080_Init(void);

/**
 * @brief Th?c hi?n do và d?c giá tr? RAW c?a Nhi?t d?.
 * @return Giá tr? RAW 16-bit c?a nhi?t d? (0xFFFF n?u l?i).
 */
uint16_t HDC1080_ReadTemperatureRaw(void);

/**
 * @brief Th?c hi?n do và d?c giá tr? RAW c?a Ð? ?m.
 * @return Giá tr? RAW 16-bit c?a d? ?m (0xFFFF n?u l?i).
 */
uint16_t HDC1080_ReadHumidityRaw(void);

// ===================================
// 4. KHAI BÁO HÀM CHUY?N Ð?I 
// ===================================

/**
 * @brief Chuy?n d?i giá tr? RAW c?a nhi?t d? sang d? Celsius (C).
 * @param raw_temp Giá tr? 16-bit d?c du?c t? c?m bi?n.
 * @return Nhi?t d? th?c t? ? d?ng float.
 */
float HDC1080_ConvertTemperature(uint16_t raw_temp);

/**
 * @brief Chuy?n d?i giá tr? RAW c?a d? ?m sang d? ?m tuong d?i (%).
 * @param raw_humi Giá tr? 16-bit d?c du?c t? c?m bi?n.
 * @return Ð? ?m tuong d?i th?c t? ? d?ng float.
 */
float HDC1080_ConvertHumidity(uint16_t raw_humi);
void delay_ms(uint32_t ms);
void HDC1080_ReadTempHumiRaw(uint16_t *raw_temp, uint16_t *raw_humi);
#endif // __HDC_1018_H