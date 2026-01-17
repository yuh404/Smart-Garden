#include <WiFi.h>
#include <PubSubClient.h> 
#include <HardwareSerial.h> 
#include <string.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>


// -------- MQTT TOPICS --------
#define TOPIC_WATER_ALL             "plant/control/water_all"
#define TOPIC_WATER_1               "plant/control/water_1"
#define TOPIC_WATER_2               "plant/control/water_2"
#define TOPIC_THRESHOLD_SOIL_1      "plant/schedule/soil/soil_1"
#define TOPIC_THRESHOLD_SOIL_2      "plant/schedule/soil/soil_2"
#define TOPIC_STATUS_PUB            "plant/status"
#define TOPIC_REFILL                "plant/control/refill"
#define TOPIC_AUTO_WATER_1          "plants/auto/water_1"
#define TOPIC_AUTO_WATER_2          "plants/auto/water_2"



#define BAUD_RATE 115200
#define RX_BUFFER_SIZE 512

// ************************************************
// 1. CAU HINH WIFI & MQTT (THAY THE CAC GIA TRI NAY)
// ************************************************
const char* ssid = ""; 
const char* password = ""; 

// ⚠️ THAY THE IP/DOMAIN CUA BAN
const char* mqtt_server = ""; // Ví dụ: IP của Mosquitto Broker
const int mqtt_port = 1883; 
const char* mqtt_user = ""; 
const char* mqtt_password = ""; 
const char* mqtt_client_id = "ESP32_GATEWAY_STM32"; 

// Topic 
const char mqtt_topic_sub[] = "plant/control/water_all"; // Topic để nhận lệnh điều khiển (T? Cloud -> ESP32)
const char mqtt_topic_pub[] = "plant/control/water_all/status"; // Topic để gửi dữ liệu/trạng thái (ESP32 -> Cloud)
const char mqtt_topic_report[] = "plant/report";
String pending_action = "";
String pending_plant = "";
String pending_motor_mode = "";
String pending_water = "";
int pending_target_moisture = 0;
bool waiting_stm32_ack = false;

// --- CAU HINH SERIAL (GIAO TIEP VOI STM32) ---
// ⚠️ Đảm bảo STM32 cũng dùng 9600 bps

HardwareSerial& stm32_uart = Serial2; // Sử dụng UART2: RX=16, TX=17

// --- LOGIC NH?N/G?I D? LI?U T? STM32 ---

char rx_buffer[RX_BUFFER_SIZE];
int rx_index = 0;
bool receiving = false;

// --- CAC BIEN QUAN LY THOI GIAN NON-BLOCKING ---
const long mqtt_reconnect_interval = 5000; 
unsigned long last_mqtt_reconnect_attempt = 0; 
const long wifi_check_interval = 10000; 
unsigned long last_wifi_check_time = 0;
unsigned long last_data_publish_time = 0;
// Giới hạn tần suất Publish dữ liệu từ STM32 lên MQTT (5 giây/lần)
const long DATA_PUBLISH_INTERVAL_MS = 5000; 
bool pending_auto_value = false;

// Khai bao cac doi tuong
WiFiClient espClient;
PubSubClient client(espClient);


// ************************************************
// 2. CAC HAM HO TRO (WIFI, SERIAL & UTILITY)
// ************************************************
String serializeToString(JsonDocument& doc) {
    String json;
    serializeJson(doc, json);
    return json;
}

void initTime() {
    configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov"); // GMT+7
}

void printWiFiDisconnectReason(uint8_t reason) {
    Serial.print("Ly do ngat ket noi: ");
    switch (reason) {
        case WIFI_REASON_NO_AP_FOUND: Serial.println("Khong tim thay AP"); break;
        case WIFI_REASON_AUTH_FAIL: Serial.println("Xac thuc that bai"); break;
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: Serial.println("Sai Mat Khau"); break;
        default: Serial.printf("Ma ly do khac: %d\n", reason); break;
    }
}

void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.println("WiFi Event: NGAT KET NOI!");
            printWiFiDisconnectReason(info.wifi_sta_disconnected.reason);
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.println("WiFi Event: KET NOI THANH CONG!");
            Serial.print("Dia chi IP cua ESP32: ");
            Serial.println(WiFi.localIP());
            break;
        default:
            break;
    }
}

void connectToWiFi() {
    WiFi.mode(WIFI_STA);
    delay(100);

    Serial.print("Dang ket noi den ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    const int maxAttempts = 20; 
    int attempts = 0;

    while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
        delay(1000); 
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println();
        Serial.println("KET NOI THAT BAI SAU THOI GIAN CHO!");
    } else {
        Serial.println();
    }
}

void send_command_to_stm32(const String& cmd) {


    Serial2.print("#CMD");
    Serial2.print(cmd);
    Serial2.print('\n');

    Serial.print("Sending command to STM32: ");
    Serial.println(cmd);

}


// ************************************************
// 3. CAC HAM XU LY MQTT
// ************************************************

// Ham Callback: Xu ly thong diep nhan duoc tu MQTT Broker (Lệnh từ Cloud)
void callback(char* topic, byte* message, unsigned int length) {
   Serial.print("📥 MQTT ← ");
    Serial.println(topic);

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, message, length);
    if (err) {
        Serial.println("❌ JSON error");
        return;
    }

    //doc["topic"] = topic;   // GẮN topic cho STM32

    String out;
    serializeJson(doc, out);

    // Forward nguyên payload
    Serial2.print("#CMD:");
    Serial2.print(out);
    Serial2.print('\n');

    Serial.println("➡ Forwarded to STM32:");
    Serial.println(out);
    
}
// Ham ket noi lai MQTT Broker (NON-BLOCKING)
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Đang kết nối MQTT... ");

    // Client ID random
    String clientId = "ESP32-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
        Serial.println("Thành công!");

      // Đăng ký topic
            client.subscribe("plant/control/#", 1);
            client.subscribe("plant/schedule/#",1);
            Serial.println("Subscribed plant/control/#");

    } else {
        Serial.print("Lỗi, mã lỗi: ");
        Serial.println(client.state());
        Serial.println("Thử lại sau 2 giây...");
        delay(2000);
    }
  }
}



  // ------------------------
  // Giao tiep voi STM32
  // ------------------------



void handle_data_packet(char* json_str) {
    // 1. Parse JSON để lấy trường "type"
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, json_str);

    if (error) {
        Serial.print("❌ JSON Parse Error: ");
        Serial.println(error.f_str());
        return;
    }
    
    // 2. Lấy type để phân loại topic
    const char* type = doc["type"];
    if (type == NULL) return; 

    // 3. Phân loại và Publish
    if (strcmp(type, "plants") == 0) {
        client.publish("plants/sensor/data", json_str);
        Serial.println("📤 MQTT → plants/sensor/data");
        Serial.println(json_str); // In trực tiếp chuỗi gốc nhận được
    } 

    else if (strcmp(type, "auto_water_done") == 0) {
        client.publish("plants/report/auto_water/data", json_str);
        Serial.println("📤 MQTT → plants/report/auto_water/data");
        Serial.println(json_str); // In trực tiếp chuỗi gốc nhận được
    }

    else if (strcmp(type, "plant") == 0) {
        client.publish("plant/sensor/data", json_str);
        Serial.println("📤 MQTT → plant/sensor/data");
        Serial.println(json_str); // In trực tiếp chuỗi gốc nhận được
    }


}


void handle_stm32_ack(char* json) {

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, json);

    if (err) {
        Serial.print("❌ JSON parse error: ");
        Serial.println(err.c_str());
        return;
    }

    const char* type = doc["type"];
    if (!type) {
        Serial.println("❌ Missing type field");
        return;
    }

    String payload;
    serializeJson(doc, payload);

    // =========================
    // CREATE PLANT ACK
    // =========================
    if (strcmp(type, "create_plant_ack") == 0) {

        client.publish("plants/report/create/status", payload.c_str());

        Serial.println("📤 MQTT → plants/report/create/status");
        Serial.println(payload);
    }

    // =========================
    // AUTO MODE ACK
    // =========================
    else if (strcmp(type, "auto_ack") == 0) {

        client.publish("plants/report/auto/status", payload.c_str());

        Serial.println("📤 MQTT → plants/report/auto/status");
        Serial.println(payload);
    }

    // =========================
    // DELETE PLANT ACK
    // =========================
    else if (strcmp(type, "delete_plant_ack") == 0) {

        client.publish("plants/report/delete/status", payload.c_str());

        Serial.println("📤 MQTT → plants/report/delete/status");
        Serial.println(payload);
    }

    // =========================
    // WATER ONCE ACK
    // =========================
    else if (strcmp(type, "water_once_ack") == 0) {

        client.publish("plant/report/water/status", payload.c_str());

        Serial.println("📤 MQTT → plant/report/water/status");
        Serial.println(payload);
    }

    // =========================
    // WATER ALL ACK
    // =========================
    else if (strcmp(type, "water_all_ack") == 0) {

        client.publish("plant/report/control/water_all/status", payload.c_str());

        Serial.println("📤 MQTT → water_all/status");
        Serial.println(payload);
    }

    // =========================
    // REFILL
    // =========================
    else if (strcmp(type, "refill_ack") == 0) {

        client.publish("plant/report/control/refill/status", payload.c_str());

        Serial.println("📤 MQTT → plant/report/control/refill/status");
        Serial.println(payload);
    }

    // =========================
    // Motor
    // =========================
    else if (strcmp(type, "motor_ack") == 0) {

        client.publish("plant/report/control/motor/status", payload.c_str());

        Serial.println("📤 MQTT → plant/report/control/motor/status");
        Serial.println(payload);
    }


    // =========================
    // SOIL
    // =========================
    else if (strcmp(type, "soil_ack") == 0) {

        client.publish("plant/report/schedule/soil/status", payload.c_str());

        Serial.println("📤 MQTT → plant/report/schedule/soil/status");
        Serial.println(payload);
    }

    // =========================
    // SET WATER
    // =========================
    else if (strcmp(type, "set_water_ack") == 0) {

        client.publish("plant/report/schedule/water_amount/status", payload.c_str());

        Serial.println("📤 MQTT → plant/report/schedule/water_amount/status");
        Serial.println(payload);
    }

    // =========================
    // SENSOR DATA
    // =========================
    else if (strcmp(type, "sensor_data") == 0) {

        client.publish("plants/report/sensor/data", payload.c_str());

        Serial.println("📤 MQTT → plants/report/sensor/data");
        Serial.println(payload);
    }

    else {
        Serial.print("⚠ Unknown type: ");
        Serial.println(type);
    }
}








// ************************************************
// 4. SETUP VA LOOP
// ************************************************

void setup() {
    // UART0 (Serial) dùng để debug (115200 bps)
    Serial.begin(115200); 
    // UART2 (stm32_uart) dùng để giao tiếp với STM32 (9600 bps)
    
    Serial2.begin(BAUD_RATE, SERIAL_8N1, 16, 17); // UART2: RX=16, TX=17
    Serial.println("ESP32 Receiver Started");
    // Thiet lap MQTT
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    initTime();
    // Thiet lap Wi-Fi
    WiFi.onEvent(WiFiEvent);
    connectToWiFi(); 
    
    last_mqtt_reconnect_attempt = millis() - mqtt_reconnect_interval; 
    last_wifi_check_time = millis();
    
}
unsigned long lastMsg = 0;

void loop() {
    

    // }
 // 1. CHECK WIFI
    if (millis() - last_wifi_check_time >= wifi_check_interval)
    {
        last_wifi_check_time = millis();

        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("⚠ WiFi lost → reconnecting...");
            connectToWiFi();
        }
    }

    // 2. MQTT reconnect
    if (!client.connected() && WiFi.status() == WL_CONNECTED)
    {
        if (millis() - last_mqtt_reconnect_attempt >= mqtt_reconnect_interval)
        {
            last_mqtt_reconnect_attempt = millis();
            reconnectMQTT();
        }
    }

    if (client.connected())
        client.loop();


    while (Serial2.available()) {
    char c = Serial2.read();

    if (c == '#') {                 // Bắt đầu frame
        rx_index = 0;
        receiving = true;
        continue;
    }

    if (receiving) {
        if (rx_index < RX_BUFFER_SIZE - 1) {

            if (c == '\n') {         // Kết thúc frame
                rx_buffer[rx_index] = '\0';
                receiving = false;

                // ==============================
                // PHÂN LOẠI GÓI TIN
                // ==============================
                if (strncmp(rx_buffer, "ACK:", 4) == 0) {
                    handle_stm32_ack(rx_buffer+4);
                }
                else if (strncmp(rx_buffer, "DATA:", 5) == 0) {
                    handle_data_packet(rx_buffer + 5); // bỏ "DATA:"
                }
                else {
                    Serial.print("⚠ Unknown packet: ");
                    Serial.println(rx_buffer);
                }

                rx_index = 0;
            }
            else {
                rx_buffer[rx_index++] = c;
            }
        }
        else {
            receiving = false;
            rx_index = 0;
        }
    }
}

}