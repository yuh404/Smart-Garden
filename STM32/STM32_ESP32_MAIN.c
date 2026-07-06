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

/* --- FreeRTOS --- */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

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

/* Chu ky cac task (ms) */
#define SENSOR_READ_INTERVAL_MS   5000
#define TELEMETRY_SEND_INTERVAL_MS 60000
#define PUMP_TASK_PERIOD_MS       100

/* ========================================================================== */
/* GLOBAL VARIABLES                             */
/* ========================================================================== */
extern DeviceSettings_t current_settings;
plant plant_list[MAX_PLANTS];

const uint8_t DISTANCE_MIN = 10;
const uint8_t DISTANCE_MAX = 50;
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
/* RTOS OBJECTS                              */
/* ========================================================================== */
/* Bao ve toan bo du lieu dung chung: plant_list[], current_settings,
   filtered_distance, volumn, is_refilling, is_water_all_active...        */
static SemaphoreHandle_t xPlantDataMutex;
/* Bao ve viec goi USART1_SendString() tu nhieu task cung luc            */
static SemaphoreHandle_t xUartTxMutex;
/* ISR UART day chuoi lenh da nhan hoan chinh vao day, Task_CommandProcess doc ra */
static QueueHandle_t xUartRxQueue;

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
static void Safe_UART_Send(const char *str);

/* RTOS task functions */
static void Task_SensorRead(void *pvParameters);
static void Task_CommandProcess(void *pvParameters);
static void Task_PumpControl(void *pvParameters);
static void Task_DataSend(void *pvParameters);

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
    HCSR04_Init();
    HDC1080_Init();

	/* LUU Y: KHONG goi SysTick_Init() nua - FreeRTOS (port.c) se tu cau hinh
	   SysTick lam nguon tick rieng cho no khi vTaskStartScheduler() chay.
	   Ham Get_Current_Time_MS() (trong systick.c) can duoc sua lai de tra ve
	   xTaskGetTickCount() * portTICK_PERIOD_MS thay vi dem SysTick thu cong. */

	/* Bat buoc: dat toan bo 4 bit NVIC cho preemption priority TRUOC khi
	   cau hinh bat ky ngat nao dung API FreeRTOS tu ISR (vd USART1_IRQHandler) */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* Uu tien phai >= 5 (so lon hon = uu tien thap hon) vi ISR nay se goi
	   xQueueSendFromISR(), phai yeu hon configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY */
	NVIC_Config_USART1(5, 0);
	/* TIM2 (dung boi HCSR04) khong goi API FreeRTOS nen giu nguyen uu tien cao */
	NVIC_Config_TIM2(0, 0);

	/* Tao cac doi tuong dong bo hoa TRUOC khi tao task va TRUOC khi cho phep ngat dung chung */
	xPlantDataMutex = xSemaphoreCreateMutex();
	xUartTxMutex    = xSemaphoreCreateMutex();
	xUartRxQueue    = xQueueCreate(5, RX_BUFFER_SIZE);

	if (xPlantDataMutex == NULL || xUartTxMutex == NULL || xUartRxQueue == NULL) {
		/* Khong du RAM de tao doi tuong RTOS - dung lai o day de de debug */
		while (1) {}
	}

	for (int i = 0; i < MAX_PLANTS; i++) {
        memcpy(&plant_list[i], &current_settings.plants[i], sizeof(plant));
        if (plant_list[i].is_active) {
            plant_count++;
        }
    }

	Safe_UART_Send("#SYSTEM INIT OK\n");

	/* Stack size tinh theo WORD (4 byte/word tren Cortex-M3), khong phai byte */
	xTaskCreate(Task_SensorRead,     "Sensor", 128, NULL, 2, NULL);
	xTaskCreate(Task_CommandProcess, "Cmd",    160, NULL, 3, NULL);
	xTaskCreate(Task_PumpControl,    "Pump",   128, NULL, 2, NULL);
	xTaskCreate(Task_DataSend,       "Send",   128, NULL, 1, NULL);

	vTaskStartScheduler();

	/* Khong bao gio toi day, tru khi het heap luc scheduler khoi dong */
	while (1) {}
}

/* ========================================================================== */
/* RTOS TASKS                                */
/* ========================================================================== */

static void Task_SensorRead(void *pvParameters) {
    (void)pvParameters;
    uint16_t raw_temp = 0, raw_humi = 0;

    for (;;) {
        raw_temp = HDC1080_ReadTemperatureRaw();
        HCSR04_Trigger();
        vTaskDelay(pdMS_TO_TICKS(30)); /* thay cho Delay_ms(30) - nhuong CPU thay vi busy-wait */

        if (xSemaphoreTake(xPlantDataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
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
            current_settings.raw_temp = HDC1080_ConvertTemperature(raw_temp);
            current_settings.raw_humi = HDC1080_ConvertHumidity(raw_humi);
            volumn = ((0.5 - Distance * 0.01) * 3.14 * 0.04 * 0.04) * 1000000;

            xSemaphoreGive(xPlantDataMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}

static void Task_CommandProcess(void *pvParameters) {
    (void)pvParameters;
    char cmd_buf[RX_BUFFER_SIZE];

    for (;;) {
        if (xQueueReceive(xUartRxQueue, cmd_buf, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(xPlantDataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                ProcessCommand(cmd_buf);
                xSemaphoreGive(xPlantDataMutex);
            }
        }
    }
}

static void Task_PumpControl(void *pvParameters) {
    (void)pvParameters;

    for (;;) {
        if (xSemaphoreTake(xPlantDataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            for (int i = 0; i < MAX_PLANTS; i++) {
                float time_needed = (float)plant_list[i].water_amount / PUMP_FLOW_RATE;
                HandleWaterOnce(&plant_list[i], plant_list[i].current_dirt, plant_list[i].threshold, plant_list[i].pump_pin, time_needed);
                HandleAutoWatering(&plant_list[i], plant_list[i].current_dirt, plant_list[i].threshold, plant_list[i].pump_pin);
            }
            HandleWaterAllLogic();
            water_refill();
            xSemaphoreGive(xPlantDataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(PUMP_TASK_PERIOD_MS));
    }
}

static void Task_DataSend(void *pvParameters) {
    (void)pvParameters;
    /* tx_buffer la static, khong nam tren stack cua task -> tiet kiem RAM dang ke
       (RAM chi 20KB nen tranh de buffer 512 byte lam bien local trong task) */
    static char tx_buffer[512];
    static char plant_line[128];

    /* snapshot du lieu can gui, lay ra khoi vung nho dung chung truoc khi format+gui,
       de khong giu xPlantDataMutex trong luc goi UART (tranh giu 2 mutex cung luc) */
    float snap_temp, snap_humi, snap_lux, snap_dist, snap_volumn;
    int   snap_plant_count;
    char  snap_id[MAX_PLANTS][20];
    float snap_soil[MAX_PLANTS];
    float snap_threshold[MAX_PLANTS];
    int   snap_water_amount[MAX_PLANTS];
    bool  snap_active[MAX_PLANTS];

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_SEND_INTERVAL_MS));

        if (xSemaphoreTake(xPlantDataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            snap_temp = current_settings.raw_temp;
            snap_humi = current_settings.raw_humi;
            snap_lux  = current_settings.lux;
            snap_dist = filtered_distance;
            snap_volumn = volumn;
            snap_plant_count = plant_count;
            for (int i = 0; i < MAX_PLANTS; i++) {
                snap_active[i] = plant_list[i].is_active;
                strncpy(snap_id[i], plant_list[i].plant_id, sizeof(snap_id[i]) - 1);
                snap_id[i][sizeof(snap_id[i]) - 1] = '\0';
                snap_soil[i] = plant_list[i].current_dirt;
                snap_threshold[i] = plant_list[i].threshold;
                snap_water_amount[i] = plant_list[i].water_amount;
            }
            xSemaphoreGive(xPlantDataMutex);
        } else {
            continue; /* khong lay duoc du lieu ky nay thi bo qua, cho ky sau */
        }

        sprintf(tx_buffer, "#DATA:{\"type\":\"plants\",\"temp\":%.1f,\"humi\":%.1f,\"lux\":%.1f,\"dist\":%.1f,\"p_count\":%d,\"volumn\":%.1f}\n",
            snap_temp, snap_humi, snap_lux, snap_dist, snap_plant_count, snap_volumn);
        Safe_UART_Send(tx_buffer);

        for (int i = 0; i < MAX_PLANTS; i++) {
            if (!snap_active[i]) continue;
            sprintf(plant_line, "#DATA:{\"type\":\"plant\",\"id\":\"%s\",\"soil\":%.1f,\"th\":%.1f,\"w\":%d}\n",
                    snap_id[i], snap_soil[i], snap_threshold[i], snap_water_amount[i]);
            Safe_UART_Send(plant_line);
        }
    }
}

/* ========================================================================== */
/* FUNCTION DEFINITIONS                           */
/* ========================================================================== */

/* Vi task dung vTaskDelay(), Delay_ms() bay gio chi con dung cho cac doan code
   chay TRUOC khi vTaskStartScheduler() (neu can). Sau khi scheduler chay,
   LUON dung vTaskDelay() thay cho ham nay de khong chan CPU cua task khac. */
void Delay_ms(uint32_t ms) {
    uint32_t total_iterations = ms * 3600;
    for (volatile uint32_t i = 0; i < total_iterations; i++) {
        __asm("nop");
    }
}

/* Wrapper bao ve USART1_SendString() bang mutex, dung o moi noi trong file nay
   thay vi goi truc tiep USART1_SendString(), vi ham co the duoc goi tu nhieu task */
static void Safe_UART_Send(const char *str) {
    if (xUartTxMutex == NULL) {
        USART1_SendString((char*)str);
        return;
    }
    if (xSemaphoreTake(xUartTxMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        USART1_SendString((char*)str);
        xSemaphoreGive(xUartTxMutex);
    }
}

void USART1_IRQHandler(void) {
    static uint8_t local_rx_buffer[RX_BUFFER_SIZE];
    static uint8_t local_rx_index = 0;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t rx_char = USART_ReceiveData(USART1);
        if (rx_char == '\n' || rx_char == '\r') {
            if (local_rx_index > 0) {
                local_rx_buffer[local_rx_index] = '\0';
                /* Day ban sao chuoi lenh vao queue, Task_CommandProcess se nhan va xu ly */
                xQueueSendFromISR(xUartRxQueue, local_rx_buffer, &xHigherPriorityTaskWoken);
                local_rx_index = 0;
            }
        } else if (local_rx_index < RX_BUFFER_SIZE - 1) {
            local_rx_buffer[local_rx_index++] = rx_char;
        }
    }
    if (USART_GetITStatus(USART1, USART_IT_ORE) != RESET) {
        USART_ReceiveData(USART1);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
        Safe_UART_Send("#ACK:{\"type\":\"motor_ack\",\"status\":\"ok\",\"action\":\"Open\"}\n");
    }
    else if (strstr(json, "\"action\":\"Close\"")) {
        set_motor(0);
        Safe_UART_Send("#ACK:{\"type\":\"motor_ack\",\"status\":\"ok\",\"action\":\"Close\"}\n");
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
                Safe_UART_Send(buf);
            }
            else if (res == -1) Safe_UART_Send("#ERR:{\"type\":\"create_plant_err\",\"message\":\"PLANT_LIST_FULL\"}\n");
            else if (res == -2) Safe_UART_Send("#ERR:{\"type\":\"create_plant_err\",\"message\":\"PLANT_EXISTS\"}\n");
        }
    }
    else if (strstr(json, "\"action\":\"remove_plant\"")) {
        char *id_ptr = strstr(json, "\"plant_id\":\"");
        if (id_ptr) {
            sscanf(id_ptr + 12, "%[^\"]", plant_id);
            int res = Delete_Plant(plant_id);
            if (res == 0) {
                sprintf(buf, "#ACK:{\"type\":\"delete_plant_ack\",\"status\":\"ok\",\"plant_id\":\"%s\"}\n", plant_id);
                Safe_UART_Send(buf);
            } else {
                sprintf(buf, "#ERR:DELETE:{\"status\":\"error\",\"message\":\"plant_not_found\",\"plant_id\":\"%s}\n", plant_id);
                Safe_UART_Send(buf);
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
            Safe_UART_Send(buf);
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
    else { Safe_UART_Send("#ERROR:INVALID_CMD\n"); }
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
        if (elapsed >= REFILL_TIMEOUT_MS) Safe_UART_Send("#ERR:{\"type\":\"refill_error\",\"msg\":\"TIMEOUT_TANK_NOT_FULL\"}\n");
        else {
            volumn = ((0.5 - filtered_distance * 0.01) * 3.14 * 0.04 * 0.04) * 1000000;
            if (volumn < 0) volumn = 0;
            sprintf(buf, "#ACK:{\"type\":\"refill_ack\",\"status\":\"ok\",\"volumn\":%.1f}\n", volumn);
            Safe_UART_Send(buf);
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
            Safe_UART_Send(buf);
        }
    }
}

void set_auto_status(const char* plant_id, int auto_status) {
    char buf[128];
    int idx = FindPlantIndex(plant_id);
    if (idx != -1) {
        plant_list[idx].auto_water = auto_status;
        sprintf(buf, "#ACK:{\"type\":\"auto_ack\",\"status\":\"ok\",\"plant_id\":\"%s\",\"auto\":%s}\n", plant_id, auto_status ? "true" : "false");
        Safe_UART_Send(buf);
    } else { Safe_UART_Send("#ERR:PLANT_NOT_FOUND\n"); }
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
                Safe_UART_Send(buf);
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
        Safe_UART_Send(buf);
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
            Safe_UART_Send(buf);
            break;
        }
    }
    if (!found) Safe_UART_Send("#ERR:SET_AMOUNT:PLANT_NOT_FOUND\n");
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

/* ========================================================================== */
/* FREERTOS HOOKS (bat buoc phai co vi da bat trong FreeRTOSConfig.h)          */
/* ========================================================================== */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    /* Tran stack cua 1 task - dung lai de debug thay vi chay tiep sai lech */
    taskDISABLE_INTERRUPTS();
    for (;;) {}
}

void vApplicationMallocFailedHook(void) {
    /* Het heap FreeRTOS (configTOTAL_HEAP_SIZE khong du) */
    taskDISABLE_INTERRUPTS();
    for (;;) {}
}
