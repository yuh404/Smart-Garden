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



#define RX_BUFFER_SIZE 128 // Kich thuoc du cho cac lenh nhu "SET_MOISTURE:30\n"
#define PUMP_PORT GPIOB
#define PUMP_PIN_1  GPIO_Pin_1 // may bom 1
#define PUMP_PIN_2  GPIO_Pin_3 // may bom 2
#define MAX_WATERING_DURATION_MS 10000
#define PUMP_PIN_3	GPIO_Pin_4 // MAI CHE
#define PUMP_FLOW_RATE 23.0f // LUU LUONG MAY BOM
#define FILTER_SIZE 10
#define ADC_DRY 3200.0f
#define ADC_WET 1500.0f
//#define MAX_PLANTS 2




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


// Bien Global/Volatile dung de giao tiep giua ISR va while(1)
volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_index = 0;
volatile bool cmd_ready = false; 
extern DeviceSettings_t current_settings;
plant plant_list[MAX_PLANTS];

const uint8_t DISTANCE_MIN = 10;
const uint8_t DISTANCE_MAX = 50;
volatile int i;
static uint32_t last_led_toggle_time = 0;
const uint32_t LED_INTERVAL = 500; // 500ms

volatile bool is_refilling = false;
volatile bool is_plant1_auto = false;
volatile bool is_plant2_auto = false;


volatile bool is_water_all = false; // C? tr?ng thái
volatile bool is_water_all_active = false; // C? B?T bom

volatile bool is_pump_1_watering = false;
volatile bool is_pump_2_watering = false;
uint32_t pump_1_start_time = 0;
uint32_t pump_2_start_time = 0;
uint16_t PUMP_PINS[MAX_PLANTS] = {PUMP_PIN_1, PUMP_PIN_2};
uint8_t ADC_CHANNELS[MAX_PLANTS] = {1, 3}; // PA1 và PA3
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
float filtered_distance = 30.0f; // Kh?i t?o giá tr? ban d?u tùy ý
const float alpha = 0.2f;

void Delay_ms(uint32_t ms)
{
    // Tính t?ng s? l?n l?p c?n thi?t
    uint32_t total_iterations = ms * 3600;

    // Vòng l?p r?ng
    for (volatile uint32_t i = 0; i < total_iterations; i++)
    {
        // Ch?ng ng?n ch?n compiler t?i ?u hóa vòng l?p
        __asm("nop"); 
    }
}







void USART1_IRQHandler(void) 
{
    // Kiem tra ngat RXNE (Receive Data Register Not Empty)
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
    {
        uint8_t rx_char = USART_ReceiveData(USART1); 

        // Kiem tra ky tu ket thuc lenh (\n hoac \r)
        if (rx_char == '\n' || rx_char == '\r') 
        {
            if (rx_index > 0) 
            {
                rx_buffer[rx_index] = '\0'; // Them null terminator
                cmd_ready = true;          // Dat co bao hieu
            }
            rx_index = 0; // Reset index
        } 
        // Luu ky tu vao buffer
        else if (rx_index < RX_BUFFER_SIZE - 1) 
        {
            rx_buffer[rx_index++] = rx_char;
        }
    }
    
    // Kiem tra va xu ly ngat ORE (Overrun Error) neu can
    if (USART_GetITStatus(USART1, USART_IT_ORE) != RESET) 
    {
        // Xoa co ORE bang cach doc USART_DR
        USART_ReceiveData(USART1); 
    }
}

void ProcessCommand(const char *command_str) 
{

// B? qua các ký t? kho?ng tr?ng/k?t thúc dòng ? d?u
    while (*command_str == ' ' || *command_str == '\r' || *command_str == '\n') {
        command_str++;
    }

    // Yêu c?u: L?nh ph?i b?t d?u b?ng "#CMD:" (Luu ý ESP32 g?i thêm d?u ':')
    if (strncmp(command_str, "#CMD:", 5) != 0) {
        return;
    }

    // Tr? d?n ph?n n?i dung JSON sau "#CMD:"
    const char *json = command_str + 5; 

    // Bi?n t?m d? phân tích cú pháp
    char plant_id[20];
    char action[32];
    int target;
    int auto_status;
    int water_amount;
    char buf[128]; // Buffer d? g?i ph?n h?i
    
    // --- 1. X? lý các L?nh Ðon gi?n (So sánh chu?i) ---
    
    // #CMDREFILL
    if (strstr(json, "\"action\":\"refill\"")){
        is_refilling = true;
        // G?i ph?n h?i ACK (c?n kh?p v?i logic x? lý ACK ? ESP32)
        // D?ng: #ACK:REFILL:OK\n
				refill_start_time = Get_Current_Time_MS();
        
    }
//    // #CMDWATER_ALL
     else if (strstr(json, "\"action\":\"water_all\"")) {
			 for(int i=0; i<MAX_PLANTS; i++) {
				if(plant_list[i].water_now){
				return;
				}
			 }
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
				 
		
    // --- 2. X? lý các L?nh Ph?c t?p (S? d?ng sscanf) ---
    
    
    // #CMDPLANT_CREATE,name=CayHoaMai,auto=1
// --- L?NH THÊM CÂY ---
// --- L?NH THÊM CÂY (Có thêm tru?ng name) ---
else if (strstr(json, "\"action\":\"create_plant\"")) {
    char *id_ptr = strstr(json, "\"plant_id\":\"");
    char *name_ptr = strstr(json, "\"name\":\"");
    char plant_name[32] = {0}; // Bi?n t?m d? ch?a tên g?i tr? l?i

    if (id_ptr) {
        // 1. L?y plant_id
        sscanf(id_ptr + 12, "%[^\"]", plant_id);

        // 2. L?y name (n?u có)
        if (name_ptr) {
            sscanf(name_ptr + 8, "%[^\"]", plant_name);
        } else {
            strcpy(plant_name, "Unknown"); // D? phòng n?u không tìm th?y tru?ng name
        }

        // 3. Th?c hi?n thêm cây vào m?ng (dùng ID)
        int res = Add_Plant(plant_id);

        if (res == 0) {
						//SyncAndSaveSettings();
            // 4. G?i ACK kèm theo c? plant_id và name nhu yêu c?u
            sprintf(buf, "#ACK:{\"type\":\"create_plant_ack\",\"status\":\"ok\",\"plant_id\":\"%s\",\"name\":\"%s\"}\n", 
                    plant_id, plant_name);
            USART1_SendString(buf);
        }
        else if (res == -1) {
            USART1_SendString("#ERR:{\"type\":\"create_plant_err\",\"message\":\"PLANT_LIST_FULL\"}\n");
        }
        else if (res == -2) {
            USART1_SendString("#ERR:{\"type\":\"create_plant_err\",\"message\":\"PLANT_EXISTS\"}\n");
        }
    }
}

    // --- L?NH XÓA CÂY ---
			else if (strstr(json, "\"action\":\"remove_plant\"")) {
        char *id_ptr = strstr(json, "\"plant_id\":\"");
        if (id_ptr) {
            // Trích xu?t giá tr? plant_id (b?t d?u sau 12 ký t? c?a c?m "plant_id":" )
            sscanf(id_ptr + 12, "%[^\"]", plant_id);
            
            int res = Delete_Plant(plant_id);
            
            if (res == 0) {
							//SyncAndSaveSettings();
                // Ph?n h?i thành công d?ng JSON
							sprintf(buf, "#ACK:{\"type\":\"delete_plant_ack\",\"status\":\"ok\",\"plant_id\":\"%s\"}\n", plant_id);
                USART1_SendString(buf);
            } else {
                // Ph?n h?i l?i n?u không tìm th?y cây
                sprintf(buf, "#ERR:DELETE:{\"status\":\"error\",\"message\":\"plant_not_found\",\"plant_id\":\"%s}\n", plant_id);
                USART1_SendString(buf);
            }
        }
    }
    
//    // #CMDSET_AUTO,plant=p001,auto=0
			else if (strstr(json, "\"plant_id\":\"") && strstr(json, "\"auto\":")) {
        // Trích xu?t plant_id
        char *id_ptr = strstr(json, "\"plant_id\":\"") + 12;
        sscanf(id_ptr, "%[^\"]", plant_id);

        // Trích xu?t auto status
        auto_status = (strstr(json, "\"auto\":true") != NULL) ? 1 : 0;

        set_auto_status(plant_id, auto_status);
    }

//    // #CMDWATER_ONCE,plant=p001
//    // D?ng l?nh don gi?n: plant=%s
    else if (strstr(json, "\"action\":\"water_once\"")) {
    char *id_ptr = strstr(json, "\"plant_id\":\""); // Tìm v? trí key
    if (id_ptr) {
        // Nh?y qua 12 ký t? c?a chu?i "plant_id":" d? l?y giá tr?
        sscanf(id_ptr + 12, "%[^\"]", plant_id); 
        
        if (is_water_all_active) {
            return; // T? ch?i n?u dang tu?i t?t c?
        }
    water_once(plant_id);
        
    }}
//    
//    // #CMDSET_SOIL,plant=p001,target=45
    else if (strstr(json, "\"targetMoisture\":")) {
        // Trích xu?t plantId (Luu ý ch? I vi?t hoa theo payload docker)
        char *id_ptr = strstr(json, "\"plant_id\":\"") + 12;
        sscanf(id_ptr, "%[^\"]", plant_id);

        // Trích xu?t targetMoisture
        char *target_ptr = strstr(json, "\"targetMoisture\":") + 17;
        sscanf(target_ptr, "%d", &target);

        if (target >= 0 && target <= 100) {
            set_soil_schedule(plant_id, target);
					sprintf(buf, "#ACK:{\"type\":\"soil_ack\",\"status\":\"ok\",\"plantId\":\"%s\",\"targetMoisture\":%d}\n", 
                    plant_id, target);
            USART1_SendString(buf);
        }
    }
		
else if (strstr(json, "\"water_amount\":")) {
        char *id_ptr = strstr(json, "\"plant_id\":\"");
        char *amount_ptr = strstr(json, "\"water_amount\":");
        
        if (id_ptr && amount_ptr) {
            // 1. Trích xu?t plant_id (b?t d?u sau 12 ký t? c?a c?m "plant_id":" )
            sscanf(id_ptr + 12, "%[^\"]", plant_id);
            
            // 2. Trích xu?t water_amount (s? nguyên)
            sscanf(amount_ptr + 15, "%d", &water_amount);
            
            if (water_amount >= 0) {
                // 3. G?i hàm th?c thi logic
                set_water_amount_schedule(plant_id, water_amount);
                
                // 4. Ph?n h?i ACK d?ng JSON
                
            }
        }
    }
    
    // --- X? lý L?i/L?nh không Kh?p ---
    else {
        // G?i ph?n h?i báo l?i
        USART1_SendString("#ERROR:INVALID_CMD\n"); 
    }
}



float HDC1080_ConvertTemperature(uint16_t raw_temp)
{
    // Công th?c: T(°C) = (Raw_Temp / 2^16) * 165 - 40
    return ((float)raw_temp / 65536.0f) * 165.0f - 40.0f;
}

/**
 * @brief Chuy?n d?i giá tr? RAW c?a d? ?m sang d? ?m tuong d?i (%)
 * @param raw_humi Giá tr? 16-bit d?c du?c t? c?m bi?n
 * @return Ð? ?m tuong d?i th?c t? ? d?ng float
 */
float HDC1080_ConvertHumidity(uint16_t raw_humi)
{
    // Công th?c: H(%) = (Raw_Humi / 2^16) * 100
    return ((float)raw_humi / 65536.0f) * 100.0f;
}


bool HDC1080_Check(void)
{
    // Thao tác ki?m tra d?a ch? ch? c?n g?i d?a ch? ? ch? d? Transmitter (Ghi)
    bool status = I2C_StartAndSendAddr(HDC1080_ADDR, I2C_Direction_Transmitter);
    
    // Luôn ph?i phát STOP sau khi ki?m tra d?a ch?
    I2C_GenerateSTOP(I2C1, ENABLE); 
    
    return status; 
}





int main(void){
	
		RCC->APB2ENR |= (1 << 4); // Bit 4 cho GPIOC
		RCC->APB2ENR |= (1 << 3);
		RCC->APB2ENR |= (1 << 2);

	
    // PC13
    GPIOC->CRH &= ~(0xF << 20); 
    GPIOC->CRH |= (0x3 << 20); 
		GPIOC->BSRR = (1 << 13);
	
	
	//PB1
    GPIOB->CRL &= ~(0xF << 4); // Xóa 4 bit c?u hình cu cho PB1 (Bit 4-7)
    GPIOB->CRL |= (0x3 << 4);

		
	//PB0
		GPIOB->CRL &= ~(0xF << 0);    
		GPIOB->CRL |= (0x1 << 0);     

	
	//PB3
		GPIOB->CRL &= ~(0xF << 12);    
		GPIOB->CRL |= (0x3 << 12);     

	//PB4
		GPIOB->CRL &= ~(0xF << 16);    
		GPIOB->CRL |= (0x3 << 16); 
		//GPIO_SetBits(PUMP_PORT, PUMP_PIN_3);
//		GPIO_ResetBits(PUMP_PORT, PUMP_PIN_3);
		GPIO_SetBits(PUMP_PORT, PUMP_PIN_1);
		GPIO_SetBits(PUMP_PORT, PUMP_PIN_2);
	
	SYSCLK_INIT();
	USART1_Config();
	
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; 

// Vô hi?u hóa ch?c nang JTAG (ch? gi? l?i SWD)
// Giá tr? 0b010 tuong ?ng v?i SWD Debug Port
	AFIO->MAPR &= ~(0x7 << 24); 

// Thi?t l?p giá tr? 0b010 (JTAG Disabled/SWD Enabled) (Giá tr? = 0x2000000)
// Thay th? AFIO_MAPR_SWJ_CFG_JTAGDISABLE b?ng giá tr?:
	AFIO->MAPR |= (0x2 << 24);
	//Settings_Load();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	ADC1_Register_Init_Calibrated();
	I2C2_Init();
	BH1750_Init();	
	I2C1_Init();     
	SysTick_Init(72000000);
//	TIMER2_INIT();
 HCSR04_Init();
  HDC1080_Init();
	bool sensor_available = false;
//	sensor_available = HDC1080_Check();

//    if (sensor_available) {
//        // N?u nh?n du?c ACK, kh?i t?o c?m bi?n
//        HDC1080_Init();
//        USART1_SendString("#SENSOR_ACK_OK. Starting data acquisition.\n");
//    } else {
//        USART1_SendString("#SENSOR_NACK_ERROR. Check wiring/pull-ups/clock.\n");
//    }
	
	
	NVIC_Config_USART1(2,0);
	NVIC_Config_TIM2(0, 0);
	USART1_SendString("#SYSTEM INIT OK\n");
	
	
	uint32_t last_read_time = 0;
	uint32_t last_send_time = 0;
	uint32_t last_read_mos = 0;
	const uint32_t READ_INTERVAL_MS = 5000;		// Doc cam bien moi 3s
	const uint32_t SEND_INTERVAL_MS = 60000; // Gui du lieu tong hop moi 15s
	const uint32_t READ_MOS_MS = 3000;
	char tx_buffer[512];
	uint16_t adc_results[2];
	float lux;
	uint16_t raw_temp= 0 , raw_humi= 0;
  float temp_c = 0.0f, humi_percent = 0.0f;
	int temp_int;
	int temp_frac; // In 2 ch? s? th?p phân (%.02d)
	int humi_int;
	int humi_frac;
	float V1;
	float V2;

//	Add_Plant("p001");
//  Add_Plant("p002");
	for (int i = 0; i < MAX_PLANTS; i++) {
        // Copy t?ng cây t? settings (Flash) vào list (RAM dang ch?y)
        memcpy(&plant_list[i], &current_settings.plants[i], sizeof(plant));
        
        // Ki?m tra cây nào dang ho?t d?ng d? c?p nh?t plant_count
        if (plant_list[i].is_active) {
            plant_count++;
        }
    }
	while(1){
	uint32_t current_time = Get_Current_Time_MS(); 

        // A. DOC CAM BIEN (Moi 3 giay)
        if (current_time - last_read_time >= READ_INTERVAL_MS) {
            last_read_time = current_time;
						
						raw_temp = HDC1080_ReadTemperatureRaw();
						HCSR04_Trigger();
						Delay_ms(30); 
						Update_Filtered_Distance();
            // Ð?c d? ?m cho t?ng cây hi?n có
        for (int i = 0; i < MAX_PLANTS; i++) {
    uint16_t adc_val = ADC1_Read_Channel(plant_list[i].adc_channel);
    
    // Công th?c Map: (adc_val - DRY) / (WET - DRY) * 100
    float moisture = ((float)adc_val - ADC_DRY) / (ADC_WET - ADC_DRY) * 100.0f;

    // Gi?i h?n giá tr? trong kho?ng 0 - 100% d? tránh s? âm ho?c > 100
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

				

				
				
				
				
        // B. KIEM TRA VA XU LY LENH TU ESP32 (NON-BLOCKING)
        // cmd_ready duoc dat boi USART1_IRQHandler khi nhan du du lieu
        if (cmd_ready) {
            ProcessCommand((const char*)rx_buffer); 
            cmd_ready = false; 
        }

				
				
        // C. GUI DU LIEU DEN ESP32 (Moi 15 giay)
        if (current_time - last_send_time >= SEND_INTERVAL_MS) {
						last_send_time = current_time;
					
					
						char temp_buf[128];

   // --- GÓI 1: G?I D? LI?U CHUNG (Topic: plants/sensor/data) ---
    // C?u trúc: #DATA:{"type":"plants","temp":28.5,...}\n
					sprintf(tx_buffer, "#DATA:{\"type\":\"plants\",\"temp\":%.1f,\"humi\":%.1f,\"lux\":%.1f,\"dist\":%.1f,\"p_count\":%d,\"volumn\":%.1f}\n",
            current_settings.raw_temp, 
            current_settings.raw_humi, 
            current_settings.lux, 
            filtered_distance, 
            plant_count,
						volumn);
    USART1_SendString(tx_buffer);

    // --- GÓI 2: G?I D? LI?U T?NG CÂY (Topic: plant/sensor/data) ---
    // G?i m?i cây là 1 gói JSON riêng d? ESP32 d? x? lý và publish
    for (int i = 0; i < MAX_PLANTS; i++) {
				if (!plant_list[i].is_active) continue;
        sprintf(tx_buffer, "#DATA:{\"type\":\"plant\",\"id\":\"%s\",\"soil\":%.1f,\"th\":%.1f,\"w\":%d}\n",
                plant_list[i].plant_id,
                plant_list[i].current_dirt,
                plant_list[i].threshold,
                plant_list[i].water_amount );
        
        USART1_SendString(tx_buffer); // G?i ngay l?p t?c gói c?a cây này
    }
    
        }
        
				
				
				
				
				
				//Delay_ms(3000);
					for (int i = 0; i < MAX_PLANTS; i++) {
        float time_needed = (float)plant_list[i].water_amount / PUMP_FLOW_RATE;
        
        // Tu?i m?t l?n (Cu?ng b?c)
        HandleWaterOnce(&plant_list[i], plant_list[i].current_dirt, 
                        plant_list[i].threshold, plant_list[i].pump_pin, time_needed);
        
        // Tu?i t? d?ng
        HandleAutoWatering(&plant_list[i], plant_list[i].current_dirt, 
                           plant_list[i].threshold, plant_list[i].pump_pin);
    }
					HandleWaterAllLogic();
					water_refill();
					
			
    }
	}


	
	void water_refill(void){
//if (!is_refilling) return; // N?u không trong tr?ng thái refill thì thoát ngay

//    char buf[128];
//    const uint32_t REFILL_TIMEOUT_MS = 15000; // Tang lên 15s cho ch?c ch?n
//    uint32_t current_time = Get_Current_Time_MS();
//    uint32_t elapsed = current_time - refill_start_time;

//    // KI?M TRA ÐI?U KI?N D?NG
//    // D?ng khi: Ðã d?y (Distance <= DISTANCE_MIN) HO?C Quá th?i gian (elapsed >= timeout)
//    if (Distance <= DISTANCE_MIN || elapsed >= REFILL_TIMEOUT_MS) {
//        
//        GPIOB->BSRR = (1 << 0); // T?T BOM (Set Pin 0)
//        is_refilling = false;

//        if (elapsed >= REFILL_TIMEOUT_MS) {
//            // Ch? g?i l?i n?u th?c s? quá th?i gian mà chua d?y
//            USART1_SendString("#ERR:{\"type\":\"refill_error\",\"msg\":\"TIMEOUT_TANK_NOT_FULL\"}\n");
//        } else {
//            // G?i thành công khi d?y bình
//            volumn = ((0.5 - Distance * 0.01) * 3.14 * 0.04 * 0.04) * 1000000;
//            sprintf(buf, "#ACK:{\"type\":\"refill_ack\",\"status\":\"ok\",\"volumn\":%.1f}\n", volumn);
//            USART1_SendString(buf);
//        }
//        refill_start_time = 0; // Reset d? l?n sau dùng ti?p
//    } 
//    else {
//        // V?n dang trong quá trình bom
//        GPIOB->BSRR = (1 << (0 + 16)); // B?T BOM (Reset Pin 0)
//    }
if (!is_refilling) return;

    char buf[128];
    const uint32_t REFILL_TIMEOUT_MS = 20000; // Tang lên 20s cho tho?i mái
    uint32_t current_time = Get_Current_Time_MS();
    uint32_t elapsed = current_time - refill_start_time;

    // C?p nh?t giá tr? l?c m?i khi hàm này du?c g?i
    Update_Filtered_Distance();

    // KI?M TRA ÐI?U KI?N D?NG d?a trên giá tr? ÐÃ L?C
    if (filtered_distance <= DISTANCE_MIN || elapsed >= REFILL_TIMEOUT_MS) {
        
        GPIOB->BSRR = (1 << 0); // T?T BOM
        is_refilling = false;

        if (elapsed >= REFILL_TIMEOUT_MS) {
            USART1_SendString("#ERR:{\"type\":\"refill_error\",\"msg\":\"TIMEOUT_TANK_NOT_FULL\"}\n");
        } else {
            // Tính toán th? tích d?a trên giá tr? ÐÃ L?C
            // Ð?i Distance sang mét (Distance * 0.01)
            volumn = ((0.5 - filtered_distance * 0.01) * 3.14 * 0.04 * 0.04) * 1000000;
            if (volumn < 0) volumn = 0; // Tránh giá tr? âm do sai s? c?m bi?n

            sprintf(buf, "#ACK:{\"type\":\"refill_ack\",\"status\":\"ok\",\"volumn\":%.1f}\n", volumn);
            USART1_SendString(buf);
        }
        refill_start_time = 0;
    } 
    else {
        // V?n dang bom
        GPIOB->BSRR = (1 << (0 + 16)); 
    }
	}
	
	
	void water_all(void){
			for(int i = 0; i < MAX_PLANTS; i++) {
        // N?u cây dang t? d?ng tu?i, t?t bom dó di d? "Water All" qu?n lý l?i t? d?u
        if(plant_list[i].is_active) {
            GPIO_SetBits(PUMP_PORT, plant_list[i].pump_pin);
            plant_list[i].auto_water_start_time = 0; 
        }
    }
			is_water_all_active = true;
}
	
void HandleWaterAllLogic(void)
{
char buf[128];

if (!is_water_all_active) return;

    uint32_t current_time = Get_Current_Time_MS();
    bool all_done = true;
		const uint32_t GLOBAL_WATER_ALL_TIMEOUT = 30000;

    for (int i = 0; i < MAX_PLANTS; i++) {
				if (!plant_list[i].is_active) continue;
        float time_needed_ms = ((float)plant_list[i].water_amount / PUMP_FLOW_RATE) * 1000.0f;
        
        // N?u cây chua b?t d?u tu?i trong mode "Water All"
        // (S? d?ng m?t c? ho?c tr?ng thái riêng n?u c?n, ? dây dùng t?m water_now)
        if (plant_list[i].auto_water_start_time == 0) {
             GPIO_ResetBits(PUMP_PORT, plant_list[i].pump_pin);
             plant_list[i].auto_water_start_time = current_time;
        }

        uint32_t elapsed = current_time - plant_list[i].auto_water_start_time;
        if (elapsed < time_needed_ms) {
            all_done = false; // V?n còn ít nh?t 1 cây dang tu?i
        } else {
            GPIO_SetBits(PUMP_PORT, plant_list[i].pump_pin);
        }
    }

    if (all_done) {
        is_water_all_active = false;
        // Reset th?i gian cho các cây d? dùng cho l?n sau
        for(int i=0; i<plant_count; i++){ 
				plant_list[i].auto_water_start_time = 0;
				uint16_t adc_val = ADC1_Read_Channel(plant_list[i].adc_channel);
        DIRT_ALL[i] = ((4000.0f - adc_val) / 2500.0f) * 100.0f;
				volumn = ((0.5 - Distance * 0.01) * 3.14 * 0.04 * 0.04) * 1000000;
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
				//SyncAndSaveSettings();
        sprintf(buf, "#ACK:{\"type\":\"auto_ack\",\"status\":\"ok\",\"plant_id\":\"%s\",\"auto\":%s}\n", 
                plant_id, auto_status ? "true" : "false");
        USART1_SendString(buf);
    } else {
        USART1_SendString("#ERR:PLANT_NOT_FOUND\n");
    }
}



void HandleAutoWatering(plant *p, float current_dirt, int threshold, uint16_t pump_pin){
		char buf[128];
		if (p->water_now || is_water_all_active|| !(p->is_active)) 
    {
//        // N?u nh? bom dang ch?y do Auto kích ho?t, ph?i T?T nó.
//        if (p->auto_water_start_time != 0) 
//        {
//             GPIO_ResetBits(PUMP_PORT, pump_pin); // T?T Bom (Active Low)
//             p->auto_water_start_time = 0;      // Reset Timer
//        }
        return; // Thoát kh?i logic Auto, nhu?ng quy?n ki?m soát cho l?nh cu?ng b?c
    }
		uint32_t current_time = Get_Current_Time_MS();
    // 1. N?u Auto B?T và KHÔNG có l?nh th? công dang ch?y
    if (p->auto_water ) 
    {
        // --- LOGIC B?T Ð?U TU?I (Khi d?t KHÔ) ---
        // B?t d?u tu?i ch? khi: Ðang khô VÀ chua B?T Ð?U tu?i
        if (current_dirt < threshold && p->auto_water_start_time == 0) 
        {
            GPIO_ResetBits(PUMP_PORT, pump_pin); // B?T Bom (Active Low)
            p->auto_water_start_time = current_time; // B?t d?u d?m
        }

        // --- LOGIC DUY TRÌ VÀ K?T THÚC TU?I ---
        // N?u bom dang ch?y (auto_water_start_time != 0)
        if (p->auto_water_start_time != 0) 
        {
            // Ði?u ki?n T?T:
            // 1. Ð? th?i gian t?i thi?u VÀ d? ?m dã Ð?T ngu?ng.
            // HO?C 2. Ðã quá th?i gian t?i da tuy?t d?i (ví d? 10 giây)
            
            bool is_duration_met = (current_time - p->auto_water_start_time >= 3000);
            bool is_soil_wet = (current_dirt >= threshold);
            
            // D?ng bom khi: (Ðã d? th?i gian và d?t dã u?t) HO?C Quá th?i gian tuy?t d?i (10s)
            if (( is_soil_wet))
            {
                GPIO_SetBits(PUMP_PORT, pump_pin); // T?T Bom (Active Low)
                p->auto_water_start_time = 0;      // Reset Timer
								uint16_t adc_val = ADC1_Read_Channel(p->adc_channel);
                float post_water_dirt = ((4000.0f - adc_val) / 2500.0f) * 100.0f;
								sprintf(buf, "#DATA:{\"type\":\"auto_water_done\",\"id\":\"%s\",\"soil\":%.1f}\n", p->plant_id, post_water_dirt);
                USART1_SendString(buf);
            }
            // N?u chua d? th?i gian, bom v?n CH?Y, b?t ch?p d? ?m dã d?t ngu?ng t?m th?i.
        }
    }
    // 2. N?u Auto T?T, d?m b?o bom T?T
    else if (!p->auto_water)
    {
         if (p->auto_water_start_time != 0) // N?u nh? dang ch?y Auto
         {
             GPIO_SetBits(PUMP_PORT, pump_pin); // T?T Bom
             p->auto_water_start_time = 0;      // Reset Timer
         }
    }
}


void water_once(const char* plant_id) {
    int idx = FindPlantIndex(plant_id);
    if (idx != -1) {
        plant_list[idx].water_now = 1;
        // Không c?n g?i ACK ? dây vì HandleWaterOnce s? g?i khi xong
    }
}


void HandleWaterOnce(plant *p, float current_dirt, int threshold, uint16_t pump_pin,float time){

char buf[128];
if (!p->water_now) return;

    uint32_t current_time = Get_Current_Time_MS();
    uint32_t duration_ms = (uint32_t)(time * 1000.0f);
		const uint32_t ABSOLUTE_TIMEOUT_MS = 30000;
    if (p->auto_water_start_time == 0) {
        GPIO_ResetBits(PUMP_PORT, pump_pin); // B?T (Active Low thì dùng Reset, tùy m?ch b?n)
        p->auto_water_start_time = current_time;
    }

    uint32_t elapsed = current_time - p->auto_water_start_time;

    // D?ng khi: Ð? th?i gian tính toán HO?C quá 30s an toàn
    if (elapsed >= duration_ms || elapsed >= ABSOLUTE_TIMEOUT_MS) {
        GPIO_SetBits(PUMP_PORT, pump_pin); // T?T
        p->auto_water_start_time = 0;
        p->water_now = 0;
				uint16_t adc_val = ADC1_Read_Channel(p->adc_channel);
        float post_water_dirt = ((4000.0f - adc_val) / 2500.0f) * 100.0f;
				volumn = ((0.5 - Distance * 0.01) * 3.14 * 0.04 * 0.04) * 1000000;
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
    if (idx != -1) {
        plant_list[idx].threshold = (float)target_moisture;
				//SyncAndSaveSettings();
    }
}


void set_motor( int mode){
	if(mode) {GPIO_ResetBits(PUMP_PORT, PUMP_PIN_3);}
	else {GPIO_SetBits(PUMP_PORT, PUMP_PIN_3);}
}


void set_water_amount_schedule(const char* plant_id, int water_amount){
		bool found = false;
    char buf[128];
    // Duy?t qua t?t c? các cây dang có trong h? th?ng
    for (int i = 0; i < plant_count; i++) {
        if (strcmp(plant_list[i].plant_id, plant_id) == 0) {
            plant_list[i].water_amount = water_amount;
						//SyncAndSaveSettings();
            found = true;
            
            // G?i ph?n h?i xác nh?n cho t?ng cây
            char msg[64];
					sprintf(buf, "#ACK:{\"type\":\"set_water_ack\",\"status\":\"ok\",\"plant_id\":\"%s\",\"water_amount\":%d}\n", 
                        plant_id, water_amount);
                USART1_SendString(buf);
            break; // Thoát vòng l?p khi dã tìm th?y
        }
    }

    if (!found) {
        USART1_SendString("#ERR:SET_AMOUNT:PLANT_NOT_FOUND\n");
    }
}
int Add_Plant(const char* id) {
    // 1. Ki?m tra xem ID dã t?n t?i trong h? th?ng chua (tránh trùng l?p)
    for (int i = 0; i < MAX_PLANTS; i++) {
        if (plant_list[i].is_active && strcmp(plant_list[i].plant_id, id) == 0) {
            return -2; // L?i: Cây dã t?n t?i
        }
    }

    // 2. Tìm v? trí (slot) tr?ng d?u tiên
    int empty_slot = -1;
    for (int i = 0; i < MAX_PLANTS; i++) {
        if (!plant_list[i].is_active) {
            empty_slot = i;
            break; // Tìm th?y ch? tr?ng d?u tiên, thoát vòng l?p
        }
    }

    // 3. N?u không còn ch? tr?ng
    if (empty_slot == -1) {
        return -1; // L?i: Danh sách d?y
    }

    // 4. Kh?i t?o thông s? t?i v? trí tr?ng tìm du?c
    memset(&plant_list[empty_slot], 0, sizeof(plant));
    strcpy(plant_list[empty_slot].plant_id, id);
    plant_list[empty_slot].threshold = 50.0f;
    plant_list[empty_slot].auto_water = 0;
    plant_list[empty_slot].water_now = 0;
    plant_list[empty_slot].water_amount = 100;
    
    // Gán c?ng Pin và ADC theo ch? s? Index c?a m?ng
    plant_list[empty_slot].pump_pin = PUMP_PINS[empty_slot];
    plant_list[empty_slot].adc_channel = ADC_CHANNELS[empty_slot];
    
    plant_list[empty_slot].is_active = true;
    plant_count++;
    
    return 0; // Thành công
}

int Delete_Plant(const char* id) {
    int idx = FindPlantIndex(id);
    if (idx != -1) {
        GPIO_SetBits(PUMP_PORT, plant_list[idx].pump_pin); // T?t bom ngay
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
    return -1; // Không tìm th?y
}

void SyncAndSaveSettings(void) {
    // Chép toàn b? m?ng cây dang ch?y vào c?u trúc luu tr?
    for (int i = 0; i < MAX_PLANTS; i++) {
        memcpy(&current_settings.plants[i], &plant_list[i], sizeof(plant));
    }
    // Ghi xu?ng Flash
    //Settings_Save();
}


float Get_Filtered_Distance(float new_val) {
    distance_buffer[filter_idx++] = new_val;
    if (filter_idx >= FILTER_SIZE) filter_idx = 0;
    
    float sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) sum += distance_buffer[i];
    return sum / FILTER_SIZE;
}

void Update_Filtered_Distance(void) {
    // Distance là bi?n t? HCSR04.h c?a b?n
    // Công th?c l?c: Y = alpha * New + (1 - alpha) * Old
    if (Distance > 2 && Distance < 400) { // Lo?i b? các giá tr? rác v?t lý (quá g?n ho?c quá xa)
        filtered_distance = (alpha * (float)Distance) + ((1.0f - alpha) * filtered_distance);
    }
}