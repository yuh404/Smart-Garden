#include "STM32_ESP32.h"
#include "SYSCLK.h"
#include "UART_CONFIG.h"
#include "setting.h"
#include "NVIC.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "systick.h"
#include "HCSR04.h"
#include "dien_dung.h"
#include "BH1750.h"
#include "HDC_1018.h"

/* ========================================================================== */
/* DEFINES                                  */
/* ========================================================================== */
#define RX_BUFFER_SIZE 128 
#define PUMP_PORT GPIOB
#define PUMP_PIN_1  GPIO_Pin_1 
#define PUMP_PIN_2  GPIO_Pin_3 
#define MAX_WATERING_DURATION_MS 10000
#define PUMP_PIN_3	GPIO_Pin_4 
#define PUMP_FLOW_RATE 23.0f 
#define FILTER_SIZE 10
#define ADC_DRY 3200.0f
#define ADC_WET 1500.0f

/* ========================================================================== */
/* GLOBAL VARIABLES                             */
/* ========================================================================== */
volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_index = 0;
volatile bool cmd_ready = false; 
extern DeviceSettings_t current_settings;
plant plant_list[MAX_PLANTS];

const uint8_t DISTANCE_MIN = 10;
const uint8_t DISTANCE_MAX = 50;
volatile int i;
static uint32_t last_led_toggle_time = 0;
const uint32_t LED_INTERVAL = 500; 

volatile bool is_refilling = false;
volatile bool is_plant1_auto = false;
volatile bool is_plant2_auto = false;
volatile bool is_water_all = false; 
volatile bool is_water_all_active = false; 

volatile bool is_pump_1_watering = false;
volatile bool is_pump_2_watering = false;
uint32_t pump_1_start_time = 0;
uint32_t pump_2_start_time = 0;
uint16_t PUMP_PINS[MAX_PLANTS] = {PUMP_PIN_1, PUMP_PIN_2};
uint8_t ADC_CHANNELS[MAX_PLANTS] = {1, 3}; 
float DIRT_ALL[2];
float volumn;
float volumn_percent;
uint32_t refill_start_time = 0;

plant plant_1 = {"p001", 1, 0,0,200};
plant plant_2 = {"p002", 1, 0,0,200};

float time_pump_plant1;
float time_pump_plant2;
int plant_count = 0;

float distance_buffer[FILTER_SIZE];
int filter_idx = 0;
float filtered_distance = 30.0f; 
const float alpha = 0.2f;

/* ========================================================================== */
/* FUNCTION PROTOTYPES                            */
/* ========================================================================== */
void water_all(void);
void water_refill(void); 
void water_once(const char* plant_id);
void set_motor( int mode);
void set_soil_schedule(const char* plant_id, int target_moisture);
void set_water_amount_schedule(const char* plant_id, int water_amount);
void set_auto_status(const char* plant_id, int auto_status);
void create_new_plant(const char* name, int auto_status);
void HandleWaterAllLogic(void);
void HandleAutoWatering(plant *p, float current_dirt, int threshold, uint16_t pump_pin);
void HandleWaterOnce(plant *p, float current_dirt, int threshold, uint16_t pump_pin,float time);
int Add_Plant(const char* id);
int Delete_Plant(const char* id);
int FindPlantIndex(const char* id);
void SyncAndSaveSettings(void);
float Get_Filtered_Distance(float new_val);
void Update_Filtered_Distance(void);
void Delay_ms(uint32_t ms);
void ProcessCommand(const char *command_str);
float HDC1080_ConvertTemperature(uint16_t raw_temp);
float HDC1080_ConvertHumidity(uint16_t raw_humi);
bool HDC1080_Check(void);

/* ========================================================================== */
/* MAIN                                    */
/* ========================================================================== */
int main(void){
	RCC->APB2ENR |= (1 << 4); 
	RCC->APB2ENR |= (1 << 3);
	RCC->APB2ENR |= (1 << 2);

	GPIOC->CRH &= ~(0xF << 20); 
	GPIOC->CRH |= (0x3 << 20); 
	GPIOC->BSRR = (1 << 13);
	
	GPIOB->CRL &= ~(0xF << 4); 
	GPIOB->CRL |= (0x3 << 4);

	GPIOB->CRL &= ~(0xF << 0);    
	GPIOB->CRL |= (0x1 << 0);      

	GPIOB->CRL &= ~(0xF << 12);    
	GPIOB->CRL |= (0x3 << 12);      

	GPIOB->CRL &= ~(0xF << 16);    
	GPIOB->CRL |= (0x3 << 16); 
	
	GPIO_SetBits(PUMP_PORT, PUMP_PIN_1);
	GPIO_SetBits(PUMP_PORT, PUMP_PIN_2);
	
	SYSCLK_INIT();
	USART1_Config();
	
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; 
	AFIO->MAPR &= ~(0x7 << 24); 
	AFIO->MAPR |= (0x2 << 24);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	ADC1_Register_Init_Calibrated();
	I2C2_Init();
	BH1750_Init();	
	I2C1_Init();      
	SysTick_Init(72000000);
    HCSR04_Init();
    HDC1080_Init();
	
	NVIC_Config_USART1(2,0);
	NVIC_Config_TIM2(0, 0);
	USART1_SendString("#SYSTEM INIT OK\n");
	
	uint32_t last_read_time = 0;
	uint32_t last_send_time = 0;
	const uint32_t READ_INTERVAL_MS = 5000;		
	const uint32_t SEND_INTERVAL_MS = 60000; 
	char tx_buffer[512];
	uint16_t raw_temp= 0 , raw_humi= 0;
    float temp_c = 0.0f, humi_percent = 0.0f;

	for (int i = 0; i < MAX_PLANTS; i++) {
        memcpy(&plant_list[i], &current_settings.plants[i], sizeof(plant));
        if (plant_list[i].is_active) {
            plant_count++;
        }
    }

	while(1){
	    uint32_t current_time = Get_Current_Time_MS(); 

        if (current_time - last_read_time >= READ_INTERVAL_MS) {
            last_read_time = current_time;
            raw_temp = HDC1080_ReadTemperatureRaw();
            HCSR04_Trigger();
            Delay_ms(30); 
            Update_Filtered_Distance();

            for (int i = 0; i < MAX_PLANTS; i++) {
                uint16_t adc_val = ADC1_Read_Channel(plant_list[i].adc_channel);
                float moisture = ((float)adc_val - ADC_DRY) / (ADC_WET - ADC_DRY) * 100.0f;
                if (moisture > 100.0f) moisture = 100.0f;
                if (moisture < 0.0f)   moisture = 0.0f;
                plant_list[i].current_dirt = moisture;
            }
					
            current_settings.lux = BH1750_ReadLux();
            raw_humi = HDC1080_ReadHumidityRaw();
            temp_c = HDC1080_ConvertTemperature(raw_temp);
            humi_percent = HDC1080_ConvertHumidity(raw_humi);
            current_settings.raw_temp = temp_c;
            current_settings.raw_humi = humi_percent;		
            volumn = ((0.5 - Distance * 0.01) * 3.14 * 0.04 * 0.04) * 1000000;
        }

        if (cmd_ready) {
            ProcessCommand((const char*)rx_buffer); 
            cmd_ready = false; 
        }

        if (current_time - last_send_time >= SEND_INTERVAL_MS) {
            last_send_time = current_time;
            sprintf(tx_buffer, "#DATA:{\"type\":\"plants\",\"temp\":%.1f,\"humi\":%.1f,\"lux\":%.1f,\"dist\":%.1f,\"p_count\":%d,\"volumn\":%.1f}\n",
                current_settings.raw_temp, current_settings.raw_humi, current_settings.lux, filtered_distance, plant_count, volumn);
            USART1_SendString(tx_buffer);

            for (int i = 0; i < MAX_PLANTS; i++) {
                if (!plant_list[i].is_active) continue;
                sprintf(tx_buffer, "#DATA:{\"type\":\"plant\",\"id\":\"%s\",\"soil\":%.1f,\"th\":%.1f,\"w\":%d}\n",
                        plant_list[i].plant_id, plant_list[i].current_dirt, plant_list[i].threshold, plant_list[i].water_amount );
                USART1_SendString(tx_buffer); 
            }
        }
        
        for (int i = 0; i < MAX_PLANTS; i++) {
            float time_needed = (float)plant_list[i].water_amount / PUMP_FLOW_RATE;
            HandleWaterOnce(&plant_list[i], plant_list[i].current_dirt, plant_list[i].threshold, plant_list[i].pump_pin, time_needed);
            HandleAutoWatering(&plant_list[i], plant_list[i].current_dirt, plant_list[i].threshold, plant_list[i].pump_pin);
        }
        HandleWaterAllLogic();
        water_refill();
    }
}

/* ========================================================================== */
/* FUNCTION DEFINITIONS                           */
/* ========================================================================== */

void Delay_ms(uint32_t ms) {
    uint32_t total_iterations = ms * 3600;
    for (volatile uint32_t i = 0; i < total_iterations; i++) {
        __asm("nop"); 
    }
}

void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t rx_char = USART_ReceiveData(USART1); 
        if (rx_char == '\n' || rx_char == '\r') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0'; 
                cmd_ready = true;          
            }
            rx_index = 0; 
        } else if (rx_index < RX_BUFFER_SIZE - 1) {
            rx_buffer[rx_index++] = rx_char;
        }
    }
    if (USART_GetITStatus(USART1, USART_IT_ORE) != RESET) {
        USART_ReceiveData(USART1); 
    }
}

void ProcessCommand(const char *command_str) {
    while (*command_str == ' ' || *command_str == '\r' || *command_str == '\n') command_str++;
    if (strncmp(command_str, "#CMD:", 5) != 0) return;
    const char *json = command_str + 5; 
    char plant_id[20];
    int target, auto_status, water_amount;
    char buf[128]; 
    
    if (strstr(json, "\"action\":\"refill\"")){
        is_refilling = true;
        refill_start_time = Get_Current_Time_MS();
    }
    else if (strstr(json, "\"action\":\"water_all\"")) {
        for(int i=0; i<MAX_PLANTS; i++) { if(plant_list[i].water_now) return; }
        water_all();
    }
    else if (strstr(json, "\"action\":\"Open\"")) {
        set_motor(1); 
        USART1_SendString("#ACK:{\"type\":\"motor_ack\",\"status\":\"ok\",\"action\":\"Open\"}\n");
    }
    else if (strstr(json, "\"action\":\"Close\"")) {
        set_motor(0); 
        USART1_SendString("#ACK:{\"type\":\"motor_ack\",\"status\":\"ok\",\"action\":\"Close\"}\n");
    }
    else if (strstr(json, "\"action\":\"create_plant\"")) {
        char *id_ptr = strstr(json, "\"plant_id\":\"");
        char *name_ptr = strstr(json, "\"name\":\"");
        char plant_name[32] = {0}; 
        if (id_ptr) {
            sscanf(id_ptr + 12, "%[^\"]", plant_id);
            if (name_ptr) sscanf(name_ptr + 8, "%[^\"]", plant_name);
            else strcpy(plant_name, "Unknown");
            int res = Add_Plant(plant_id);
            if (res == 0) {
                sprintf(buf, "#ACK:{\"type\":\"create_plant_ack\",\"status\":\"ok\",\"plant_id\":\"%s\",\"name\":\"%s\"}\n", plant_id, plant_name);
                USART1_SendString(buf);
            }
            else if (res == -1) USART1_SendString("#ERR:{\"type\":\"create_plant_err\",\"message\":\"PLANT_LIST_FULL\"}\n");
            else if (res == -2) USART1_SendString("#ERR:{\"type\":\"create_plant_err\",\"message\":\"PLANT_EXISTS\"}\n");
        }
    }
    else if (strstr(json, "\"action\":\"remove_plant\"")) {
        char *id_ptr = strstr(json, "\"plant_id\":\"");
        if (id_ptr) {
            sscanf(id_ptr + 12, "%[^\"]", plant_id);
            int res = Delete_Plant(plant_id);
            if (res == 0) {
                sprintf(buf, "#ACK:{\"type\":\"delete_plant_ack\",\"status\":\"ok\",\"plant_id\":\"%s\"}\n", plant_id);
                USART1_SendString(buf);
            } else {
                sprintf(buf, "#ERR:DELETE:{\"status\":\"error\",\"message\":\"plant_not_found\",\"plant_id\":\"%s}\n", plant_id);
                USART1_SendString(buf);
            }
        }
    }
    else if (strstr(json, "\"plant_id\":\"") && strstr(json, "\"auto\":")) {
        char *id_ptr = strstr(json, "\"plant_id\":\"") + 12;
        sscanf(id_ptr, "%[^\"]", plant_id);
        auto_status = (strstr(json, "\"auto\":true") != NULL) ? 1 : 0;
        set_auto_status(plant_id, auto_status);
    }
    else if (strstr(json, "\"action\":\"water_once\"")) {
        char *id_ptr = strstr(json, "\"plant_id\":\""); 
        if (id_ptr) {
            sscanf(id_ptr + 12, "%[^\"]", plant_id); 
            if (!is_water_all_active) water_once(plant_id);
        }
    }
    else if (strstr(json, "\"targetMoisture\":")) {
        char *id_ptr = strstr(json, "\"plant_id\":\"") + 12;
        sscanf(id_ptr, "%[^\"]", plant_id);
        char *target_ptr = strstr(json, "\"targetMoisture\":") + 17;
        sscanf(target_ptr, "%d", &target);
        if (target >= 0 && target <= 100) {
            set_soil_schedule(plant_id, target);
            sprintf(buf, "#ACK:{\"type\":\"soil_ack\",\"status\":\"ok\",\"plantId\":\"%s\",\"targetMoisture\":%d}\n", plant_id, target);
            USART1_SendString(buf);
        }
    }
    else if (strstr(json, "\"water_amount\":")) {
        char *id_ptr = strstr(json, "\"plant_id\":\"");
        char *amount_ptr = strstr(json, "\"water_amount\":");
        if (id_ptr && amount_ptr) {
            sscanf(id_ptr + 12, "%[^\"]", plant_id);
            sscanf(amount_ptr + 15, "%d", &water_amount);
            if (water_amount >= 0) set_water_amount_schedule(plant_id, water_amount);
        }
    }
    else { USART1_SendString("#ERROR:INVALID_CMD\n"); }
}

float HDC1080_ConvertTemperature(uint16_t raw_temp) { return ((float)raw_temp / 65536.0f) * 165.0f - 40.0f; }
float HDC1080_ConvertHumidity(uint16_t raw_humi) { return ((float)raw_humi / 65536.0f) * 100.0f; }

bool HDC1080_Check(void) {
    bool status = I2C_StartAndSendAddr(HDC1080_ADDR, I2C_Direction_Transmitter);
    I2C_GenerateSTOP(I2C1, ENABLE); 
    return status; 
}

void water_refill(void) {
    if (!is_refilling) return;
    char buf[128];
    const uint32_t REFILL_TIMEOUT_MS = 20000; 
    uint32_t current_time = Get_Current_Time_MS();
    uint32_t elapsed = current_time - refill_start_time;
    Update_Filtered_Distance();
    if (filtered_distance <= DISTANCE_MIN || elapsed >= REFILL_TIMEOUT_MS) {
        GPIOB->BSRR = (1 << 0); 
        is_refilling = false;
        if (elapsed >= REFILL_TIMEOUT_MS) USART1_SendString("#ERR:{\"type\":\"refill_error\",\"msg\":\"TIMEOUT_TANK_NOT_FULL\"}\n");
        else {
            volumn = ((0.5 - filtered_distance * 0.01) * 3.14 * 0.04 * 0.04) * 1000000;
            if (volumn < 0) volumn = 0; 
            sprintf(buf, "#ACK:{\"type\":\"refill_ack\",\"status\":\"ok\",\"volumn\":%.1f}\n", volumn);
            USART1_SendString(buf);
        }
        refill_start_time = 0;
    } else { GPIOB->BSRR = (1 << (0 + 16)); }
}

void water_all(void) {
    for(int i = 0; i < MAX_PLANTS; i++) {
        if(plant_list[i].is_active) {
            GPIO_SetBits(PUMP_PORT, plant_list[i].pump_pin);
            plant_list[i].auto_water_start_time = 0; 
        }
    }
    is_water_all_active = true;
}

void HandleWaterAllLogic(void) {
    char buf[128];
    if (!is_water_all_active) return;
    uint32_t current_time = Get_Current_Time_MS();
    bool all_done = true;
    for (int i = 0; i < MAX_PLANTS; i++) {
        if (!plant_list[i].is_active) continue;
        float time_needed_ms = ((float)plant_list[i].water_amount / PUMP_FLOW_RATE) * 1000.0f;
        if (plant_list[i].auto_water_start_time == 0) {
             GPIO_ResetBits(PUMP_PORT, plant_list[i].pump_pin);
             plant_list[i].auto_water_start_time = current_time;
        }
        uint32_t elapsed = current_time - plant_list[i].auto_water_start_time;
        if (elapsed < time_needed_ms) all_done = false; 
        else GPIO_SetBits(PUMP_PORT, plant_list[i].pump_pin);
    }
    if (all_done) {
        is_water_all_active = false;
        for(int i=0; i<plant_count; i++){ 
            plant_list[i].auto_water_start_time = 0;
            uint16_t adc_val = ADC1_Read_Channel(plant_list[i].adc_channel);
            DIRT_ALL[i] = ((4000.0f - adc_val) / 2500.0f) * 100.0f;
            sprintf(buf,"#ACK:{\"type\":\"water_all_ack\",\"status\":\"ok\",\"%s\":%.1f,\"volumn\":%.1f}\n",plant_list[i].plant_id, DIRT_ALL[i], 1000.0);
            USART1_SendString(buf);
        }
    }
}

void set_auto_status(const char* plant_id, int auto_status) {
    char buf[128];
    int idx = FindPlantIndex(plant_id);
    if (idx != -1) {
        plant_list[idx].auto_water = auto_status;
        sprintf(buf, "#ACK:{\"type\":\"auto_ack\",\"status\":\"ok\",\"plant_id\":\"%s\",\"auto\":%s}\n", plant_id, auto_status ? "true" : "false");
        USART1_SendString(buf);
    } else { USART1_SendString("#ERR:PLANT_NOT_FOUND\n"); }
}

void HandleAutoWatering(plant *p, float current_dirt, int threshold, uint16_t pump_pin) {
    if (p->water_now || is_water_all_active|| !(p->is_active)) return;
    uint32_t current_time = Get_Current_Time_MS();
    if (p->auto_water ) {
        if (current_dirt < threshold && p->auto_water_start_time == 0) {
            GPIO_ResetBits(PUMP_PORT, pump_pin); 
            p->auto_water_start_time = current_time; 
        }
        if (p->auto_water_start_time != 0) {
            bool is_soil_wet = (current_dirt >= threshold);
            if (is_soil_wet) {
                GPIO_SetBits(PUMP_PORT, pump_pin); 
                p->auto_water_start_time = 0; 
                uint16_t adc_val = ADC1_Read_Channel(p->adc_channel);
                float post_water_dirt = ((4000.0f - adc_val) / 2500.0f) * 100.0f;
                char buf[128];
                sprintf(buf, "#DATA:{\"type\":\"auto_water_done\",\"id\":\"%s\",\"soil\":%.1f}\n", p->plant_id, post_water_dirt);
                USART1_SendString(buf);
            }
        }
    }
    else if (!p->auto_water) {
         if (p->auto_water_start_time != 0) {
             GPIO_SetBits(PUMP_PORT, pump_pin); 
             p->auto_water_start_time = 0; 
         }
    }
}

void water_once(const char* plant_id) {
    int idx = FindPlantIndex(plant_id);
    if (idx != -1) plant_list[idx].water_now = 1;
}

void HandleWaterOnce(plant *p, float current_dirt, int threshold, uint16_t pump_pin, float time) {
    if (!p->water_now) return;
    uint32_t current_time = Get_Current_Time_MS();
    uint32_t duration_ms = (uint32_t)(time * 1000.0f);
    const uint32_t ABSOLUTE_TIMEOUT_MS = 30000;
    if (p->auto_water_start_time == 0) {
        GPIO_ResetBits(PUMP_PORT, pump_pin); 
        p->auto_water_start_time = current_time;
    }
    uint32_t elapsed = current_time - p->auto_water_start_time;
    if (elapsed >= duration_ms || elapsed >= ABSOLUTE_TIMEOUT_MS) {
        GPIO_SetBits(PUMP_PORT, pump_pin); 
        p->auto_water_start_time = 0;
        p->water_now = 0;
        uint16_t adc_val = ADC1_Read_Channel(p->adc_channel);
        float post_water_dirt = ((4000.0f - adc_val) / 2500.0f) * 100.0f;
        char buf[128];
        if (elapsed >= ABSOLUTE_TIMEOUT_MS && elapsed < duration_ms) {
            sprintf(buf, "#WARN:{\"type\":\"water_once_timeout\",\"plant_id\":\"%s\",\"msg\":\"Safety limit reached\"}\n", p->plant_id);
        } else {
            sprintf(buf, "#ACK:{\"type\":\"water_once_ack\",\"status\":\"ok\",\"plant_id\":\"%s\",\"soil\":%.1f,\"volumn\":%.1f}\n", p->plant_id, post_water_dirt, 1000.0);
        }
        USART1_SendString(buf);
    }
}

void set_soil_schedule(const char* plant_id, int target_moisture) {
    int idx = FindPlantIndex(plant_id);
    if (idx != -1) plant_list[idx].threshold = (float)target_moisture;
}

void set_motor( int mode) {
	if(mode) GPIO_ResetBits(PUMP_PORT, PUMP_PIN_3);
	else GPIO_SetBits(PUMP_PORT, PUMP_PIN_3);
}

void set_water_amount_schedule(const char* plant_id, int water_amount) {
    bool found = false;
    char buf[128];
    for (int i = 0; i < plant_count; i++) {
        if (strcmp(plant_list[i].plant_id, plant_id) == 0) {
            plant_list[i].water_amount = water_amount;
            found = true;
            sprintf(buf, "#ACK:{\"type\":\"set_water_ack\",\"status\":\"ok\",\"plant_id\":\"%s\",\"water_amount\":%d}\n", plant_id, water_amount);
            USART1_SendString(buf);
            break; 
        }
    }
    if (!found) USART1_SendString("#ERR:SET_AMOUNT:PLANT_NOT_FOUND\n");
}

int Add_Plant(const char* id) {
    for (int i = 0; i < MAX_PLANTS; i++) {
        if (plant_list[i].is_active && strcmp(plant_list[i].plant_id, id) == 0) return -2; 
    }
    int empty_slot = -1;
    for (int i = 0; i < MAX_PLANTS; i++) {
        if (!plant_list[i].is_active) { empty_slot = i; break; }
    }
    if (empty_slot == -1) return -1; 
    memset(&plant_list[empty_slot], 0, sizeof(plant));
    strcpy(plant_list[empty_slot].plant_id, id);
    plant_list[empty_slot].threshold = 50.0f;
    plant_list[empty_slot].auto_water = 0;
    plant_list[empty_slot].water_now = 0;
    plant_list[empty_slot].water_amount = 100;
    plant_list[empty_slot].pump_pin = PUMP_PINS[empty_slot];
    plant_list[empty_slot].adc_channel = ADC_CHANNELS[empty_slot];
    plant_list[empty_slot].is_active = true;
    plant_count++;
    return 0; 
}

int Delete_Plant(const char* id) {
    int idx = FindPlantIndex(id);
    if (idx != -1) {
        GPIO_SetBits(PUMP_PORT, plant_list[idx].pump_pin); 
        plant_list[idx].is_active = false;
        memset(plant_list[idx].plant_id, 0, sizeof(plant_list[idx].plant_id));
        if (plant_count > 0) plant_count--;
        return 0;
    }
    return -1;
}

int FindPlantIndex(const char* id) {
    for (int i = 0; i < MAX_PLANTS; i++) {
        if (plant_list[i].is_active && strcmp(plant_list[i].plant_id, id) == 0) return i;
    }
    return -1; 
}

void SyncAndSaveSettings(void) {
    for (int i = 0; i < MAX_PLANTS; i++) memcpy(&current_settings.plants[i], &plant_list[i], sizeof(plant));
}

float Get_Filtered_Distance(float new_val) {
    distance_buffer[filter_idx++] = new_val;
    if (filter_idx >= FILTER_SIZE) filter_idx = 0;
    float sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) sum += distance_buffer[i];
    return sum / FILTER_SIZE;
}

void Update_Filtered_Distance(void) {
    if (Distance > 2 && Distance < 400) { 
        filtered_distance = (alpha * (float)Distance) + ((1.0f - alpha) * filtered_distance);
    }
}