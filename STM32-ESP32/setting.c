// settings.c

#include "setting.h"

#include "stm32f10x_flash.h"
// Magic Number de xac nhan du lieu da duoc ghi
#define SETTINGS_MAGIC 0xAA

DeviceSettings_t current_settings;

void Settings_Load(void) {
//    // Tr? t?i vùng Flash
//    DeviceSettings_t *flash_settings = (DeviceSettings_t *)FLASH_PAGE_ADDRESS;

//    // Ki?m tra Magic Number
//    if (flash_settings->magic_number == SETTINGS_MAGIC) {
//        // Sao chép toàn b? c?u trúc t? Flash vào RAM
//        memcpy(&current_settings, flash_settings, sizeof(DeviceSettings_t));
//    } else {
//        // Kh?i t?o m?c d?nh cho các bi?n b?n quan tâm
//        current_settings.magic_number = SETTINGS_MAGIC;
//        
//        for(int i = 0; i < MAX_PLANTS; i++) {
//            // Thi?t l?p giá tr? m?c d?nh cho các bi?n thay d?i
//            current_settings.plants[i].auto_water = 0;     // M?c d?nh t?t t? d?ng
//            current_settings.plants[i].threshold = 50.0f;  // M?c d?nh 50%
//            current_settings.plants[i].water_amount = 100; // M?c d?nh 100ml
//            
//            // Các giá tr? c? d?nh ho?c kh?i t?o khác
//            sprintf(current_settings.plants[i].plant_id, "p00%d", i+1);
//            current_settings.plants[i].is_active = true;
//            current_settings.plants[i].pump_pin = (i == 0) ? GPIO_Pin_1 : GPIO_Pin_3;
//            current_settings.plants[i].adc_channel = (i == 0) ? 1 : 3;
//        }
//        
//        // Ghi giá tr? m?c d?nh vào Flash
//        Settings_Save(); 
//    }
		DeviceSettings_t *flash_settings = (DeviceSettings_t *)FLASH_PAGE_ADDRESS;

    if (flash_settings->magic_number == SETTINGS_MAGIC) {
        // N?u dã có d? li?u trong Flash, n?p vào RAM
        memcpy(&current_settings, flash_settings, sizeof(DeviceSettings_t));
    } else {
        // N?U CHUA CÓ D? LI?U (L?n d?u ch?y)
        current_settings.magic_number = SETTINGS_MAGIC;
        
        // Xóa s?ch d? li?u, coi nhu chua có cây nào
        for(int i = 0; i < MAX_PLANTS; i++) {
            current_settings.plants[i].is_active = false; // QUAN TR?NG
            memset(current_settings.plants[i].plant_id, 0, 20);
            current_settings.plants[i].auto_water = 0;
            current_settings.plants[i].water_amount = 100;
        }
        
        // Luu l?i tr?ng thái "tr?ng" này vào Flash
        Settings_Save(); 
    }
}


void Settings_Save(void) {
    FLASH_Status status = FLASH_COMPLETE;
    uint32_t Address = FLASH_PAGE_ADDRESS;
    uint32_t *Data_ptr = (uint32_t*)&current_settings; // Chuyen cau truc thanh con tro 32-bit
    uint32_t num_words = sizeof(DeviceSettings_t) / 4; // So luong tu 32-bit de ghi

    // 1. Mo khoa Flash
    FLASH_Unlock();

    // 2. Xoa trang Flash truoc khi ghi
    status = FLASH_ErasePage(Address);

    if (status == FLASH_COMPLETE) {
        // 3. Ghi du lieu tung tu (Word - 32 bit)
        for (uint32_t i = 0; (i < num_words) && (status == FLASH_COMPLETE); i++) {
            status = FLASH_ProgramWord(Address + (i * 4), *Data_ptr);
            Data_ptr++;
        }
    }

    // 4. Khoa Flash lai de bao ve
    FLASH_Lock();
    
    // Ghi chu: Can them xu ly loi neu status != FLASH_COMPLETE
}

