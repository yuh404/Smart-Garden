#ifndef __setting_H
#define __setting_H


#include <stdint.h>
#include <stdbool.h>                 // Device header
#include <string.h>
#include <stdio.h>
// Dia chi sector trong Flash de luu tru (vi du: Sector 63, o cuoi Flash)
// STM32F103C8T6 co 64KB Flash, moi trang la 1KB (1024 bytes)
// Chon trang cuoi cung (dia chi 0x0800 F800 - 0x0800 FFFF)
#define FLASH_PAGE_SIZE    ((uint16_t)0x400)      // 1024 bytes
#define FLASH_PAGE_ADDRESS ((uint32_t)0x0801FC00)// Dia chi bat dau cua trang 62 (trang cuoi cung truoc trang bootloader)
#define MAX_PLANTS         2
// Cau truc chua tat ca cac nguong can luu tru


typedef struct {
	char plant_id[10];
	int auto_water;
	float current_dirt;      
  float threshold;
	uint32_t auto_water_start_time;
	int water_now;
	int water_amount;
	uint16_t pump_pin;
  uint8_t adc_channel;
	bool is_active;
} plant;

typedef struct {
    uint16_t magic_number;    
		float lux;
		float raw_temp;
		float raw_humi;
		plant plants[MAX_PLANTS];
} DeviceSettings_t;

// Bien toan cuc chua cac thiet lap hien tai
extern DeviceSettings_t current_settings;
extern DeviceSettings_t previous_settings;
void Settings_Load(void);
void Settings_Save(void);


#endif