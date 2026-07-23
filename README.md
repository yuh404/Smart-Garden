# Smart Garden IoT System (STM32 & ESP32)

Hệ thống tưới tiêu và giám sát vườn thông minh được thiết kế theo mô hình phân tán xử lý, tận dụng sức mạnh thời gian thực của vi điều khiển kiến trúc ARM (STM32) kết hợp với khả năng kết nối không dây linh hoạt của nền tảng IoT (ESP32).

<div align="center">
  <img width="505" height="410" alt="image" src="https://github.com/user-attachments/assets/6a7c4c7c-fb04-4ec6-b86e-b01c9f962f36" />

</div>

---

## 1. Features (Tính năng hệ thống)

*   **Hệ điều hành thời gian thực (RTOS):** Ứng dụng FreeRTOS để quản lý các tác vụ đa nhiệm một cách mượt mà, bao gồm 4 luồng chính: Đọc cảm biến, Xử lý lệnh, Điều khiển bơm và Gửi dữ liệu từ xa (Telemetry).
*   **Tự động tưới cây thông minh (Auto Watering):** Liên tục theo dõi độ ẩm đất qua cảm biến điện dung và tự động bật/tắt máy bơm riêng biệt cho từng chậu cây khi độ ẩm vượt qua ngưỡng cài đặt.
*   **Điều khiển và cấu hình từ xa:** Phân tích các chuỗi lệnh định dạng JSON nhận từ ESP32 qua UART để thực hiện tưới thủ công (`water_once`, `water_all`), cập nhật lượng nước cần tưới, và bật/tắt chế độ tự động.
*   **Quản lý bồn chứa nước tự động:** Dùng cảm biến siêu âm HC-SR04 để đo lường khoảng cách mặt nước, từ đó tính toán thể tích hiện tại và tự động kích hoạt bơm cấp nước (refill) khi cạn với tính năng tự ngắt để bảo vệ mạch.
*   **Giám sát môi trường toàn diện:** Cập nhật liên tục các thông số nhiệt độ, độ ẩm (HDC1080) và cường độ ánh sáng (BH1750) để đóng gói thành chuỗi JSON và gửi định kỳ mỗi 60 giây lên hệ thống máy chủ.

---

## 2. Hardware Architecture (Kiến trúc Phần cứng)

<div align="center">
  <img width="691" height="360" alt="image" src="https://github.com/user-attachments/assets/53554623-f6f3-4011-9073-0eff872e0aec" />

</div>

*   **Core MCU (STM32F103C8T6 - Blue Pill):** Đóng vai trò là bộ não trung tâm của hệ thống. Vi điều khiển này chịu trách nhiệm chạy hệ điều hành FreeRTOS, thu thập dữ liệu từ toàn bộ mảng cảm biến, thực thi các thuật toán điều khiển máy bơm và quản lý logic tưới tiêu tự động với độ trễ cực thấp.
*   **Co-processor (ESP32):** Hoạt động như một Gateway IoT phụ trợ, giao tiếp liên tục với STM32 thông qua chuẩn UART (chân `PA9`/`PA10`). Nhiệm vụ chính của ESP32 là xử lý kết nối mạng không dây (Wi-Fi), nhận các lệnh điều khiển dạng JSON từ người dùng, đồng thời đóng gói dữ liệu môi trường (Telemetry) để đẩy lên hệ thống máy chủ thông qua giao thức MQTT và đồng bộ trực tiếp với nền tảng Firebase .
*   **Sensors (Hệ thống Cảm biến):**
    *   **BH1750 (I2C):** Cảm biến đo cường độ ánh sáng môi trường .
    *   **HDC1080 (I2C):** Cảm biến đo nhiệt độ và độ ẩm không khí với độ chính xác cao .
    *   **Cảm biến độ ẩm đất điện dung (Analog):** Theo dõi lượng nước trong đất của từng chậu cây độc lập, cung cấp thông số đầu vào để STM32 ra quyết định tưới .
    *   **RTC DS3231:** Module thời gian thực giúp hệ thống đồng bộ và duy trì lịch trình hoạt động chính xác .
    *   **HC-SR04:** Cảm biến siêu âm đo khoảng cách mặt nước, phục vụ tính năng cảnh báo và tự động kích hoạt máy bơm để cấp đầy bồn chứa .
*   **Power & Integration:** Toàn bộ cụm phần cứng này có thể được tối ưu hóa bằng cách thiết kế quy hoạch trên một mạch chung . Ngoài việc dùng adapter cấp nguồn DC cố định, kiến trúc hệ thống hiện tại cũng rất phù hợp để tích hợp thêm các trạm cấp nguồn cơ động (như module shield pin) nhằm triển khai linh hoạt tại các vị trí sân vườn không có sẵn lưới điện .

---

## 3. Software & RTOS Implementation (Kiến trúc Phần mềm)

Hệ thống được phát triển trên môi trường Keil C, sử dụng hệ điều hành thời gian thực FreeRTOS để giải quyết triệt để bài toán nghẽn cổ chai (blocking) thường gặp ở các vòng lặp `while(1)` truyền thống .
<div align="center">
  <img width="290" height="335" alt="image" src="https://github.com/user-attachments/assets/46e8d861-de98-4189-925f-8bc60cbcee8e" />
	<img width="883" height="542" alt="image" src="https://github.com/user-attachments/assets/47f807eb-b968-4ad7-944a-b2f958bad826" />

</div>



### 3.1. FreeRTOS Core Configuration
*   **Tick Rate:** FreeRTOS được cấu hình với tần số Tick là 1000Hz (độ phân giải 1ms), cho phép hệ thống định tuyến thời gian trễ và chuyển đổi ngữ cảnh (Context Switching) cực kỳ chính xác .
*   **Preemptive Scheduling:** Hỗ trợ cơ chế ưu tiên ngắt (Preemption), cho phép các tác vụ khẩn cấp (như xử lý lệnh từ người dùng) ngay lập tức chiếm quyền CPU từ các tác vụ đang chạy có mức ưu tiên thấp hơn .

### 3.2. Quản lý Đa nhiệm (Task Management)
Luồng chương trình chính được chia làm 4 Task độc lập, được phân bổ mức ưu tiên (Priority) từ 1 đến 3 :
*   `Task_SensorRead` **(Priority 2):** Chịu trách nhiệm quét định kỳ toàn bộ cảm biến . Các hàm giao tiếp I2C (HDC1080, BH1750) và đo ADC (độ ẩm đất) được gọi tuần tự tại đây, xen kẽ với hàm `vTaskDelay()` để nhường CPU cho các Task khác .
*   `Task_CommandProcess` **(Priority 3):** Task có mức ưu tiên cao nhất, luôn ở trạng thái ngủ (Blocked) và chỉ thức dậy khi nhận được dữ liệu từ hàng đợi (Queue) UART truyền sang . Nhiệm vụ của Task là phân tích ngay lập tức các chuỗi JSON (ví dụ: bật bơm, đổi cấu hình tưới) để hệ thống phản hồi tức thời .
*   `Task_PumpControl` **(Priority 2):** Liên tục kiểm tra logic thời gian và so sánh độ ẩm đất hiện tại với ngưỡng (threshold) đã cài đặt để điều khiển đóng/cắt các chân GPIO kết nối đến Relay máy bơm .
*   `Task_DataSend` **(Priority 1):** Định kỳ thu thập các bản ghi môi trường mới nhất, đóng gói thành cấu trúc JSON chuẩn và gửi lên ESP32 qua UART để đồng bộ lên Cloud .

### 3.3. Cơ chế Đồng bộ & Giao tiếp (Inter-Task Communication)
*   **Queue (Hàng đợi):** Đóng vai trò là cầu nối an toàn giữa ngắt phần cứng (`USART1_IRQHandler`) và `Task_CommandProcess` . Dữ liệu nhận từ ESP32 sẽ được đẩy vào Queue ngay trong trình phục vụ ngắt mà không làm treo hệ thống .
*   **Mutex (Khóa bảo vệ):** Các biến lưu trữ cấu hình cây trồng và thông số cảm biến dùng chung (`plant_list`, `current_settings`) được bảo vệ nghiêm ngặt bằng Mutex (`xPlantDataMutex`) . Điều này đảm bảo dữ liệu không bị sai lệch khi Task đọc cảm biến đang ghi dữ liệu mà Task gửi dữ liệu lại truy cập vào cùng lúc .

### 3.4. Xử lý Ngắt phần cứng (Hardware Interrupts)
*   **Ngắt UART1 RX:** Được cấp mức ưu tiên ngắt hợp lệ (Priority 5) để nằm trong vùng an toàn, cho phép gọi các hàm API của FreeRTOS (như `xQueueSendFromISR`) một cách hợp lệ (`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`) .
*   **Ngắt Timer 2 (TIM2):** Cấu hình hoạt động hoàn toàn độc lập với FreeRTOS (Priority 0) để phục vụ riêng cho cảm biến siêu âm HC-SR04, giúp bắt chính xác các xung dội lại mà không bị ảnh hưởng bởi độ trễ chuyển đổi Task .
*   **Lấy mẫu ADC Polling:** ADC1 được cấu hình đọc kênh đôi (`PA1` và `PA3`) bằng kích hoạt phần mềm (SWSTART) . Hệ thống sẽ kiểm tra cờ EOC (End of Conversion) để lấy giá trị chính xác trước khi tiếp tục thực thi lệnh mà không sử dụng ngắt .

---

## 4. Technical Highlights & Debugging (Điểm nhấn Kỹ thuật)

> Một trong những thách thức lớn nhất của dự án là việc giao tiếp UART giữa STM32 và ESP32 . ESP32 liên tục đẩy các chuỗi lệnh JSON có kích thước lớn xuống STM32 . Nếu không xử lý tốt, hệ thống sẽ gặp tình trạng mất gói tin, treo vi điều khiển hoặc xung đột dữ liệu . Dự án đã giải quyết triệt để các vấn đề này bằng các kỹ thuật sau :

### 4.1. Tách biệt không gian Ngắt và không gian Task (RX Handling)
*   **Vấn đề:** Xử lý chuỗi JSON trực tiếp bên trong trình phục vụ ngắt (ISR) sẽ làm hệ thống bị treo, không thể thực thi các Task khác .
*   **Giải pháp:** Ngắt `USART1_IRQHandler` chỉ đảm nhiệm vai trò rất nhẹ là thu thập từng byte vào một mảng tạm . Ngay khi phát hiện ký tự kết thúc chuỗi (`\n` hoặc `\r`), toàn bộ mảng này lập tức được đẩy vào hàng đợi `xUartRxQueue` thông qua hàm an toàn `xQueueSendFromISR()` . Việc phân tích cú pháp JSON nặng nề được nhường lại cho `Task_CommandProcess` xử lý ở không gian luồng bình thường, đảm bảo tính thời gian thực cho hệ thống .

### 4.2. Bẫy lỗi Tràn bộ đệm cứng (Overrun Error - ORE)
*   **Vấn đề:** Khi ESP32 truyền dữ liệu với tần số quá cao, CPU của STM32 không kịp đọc thanh ghi DR, dẫn đến cờ lỗi ORE (Overrun) bị bật lên . Đặc điểm của lỗi này trên lõi ARM là nếu không được xóa, ngắt UART sẽ bị treo vĩnh viễn .
*   **Giải pháp Debug:** Trong ISR, dự án đã bắt riêng trường hợp lỗi này (`USART_GetITStatus(USART1, USART_IT_ORE) != RESET`) và thực hiện thao tác đọc rỗng thanh ghi (`USART_ReceiveData(USART1)`) để xóa cờ lỗi một cách chủ động . Đây là một kỹ thuật "cứu cánh" giúp hệ thống tự phục hồi ngay lập tức khi đường truyền quá tải thay vì bị treo cứng .

### 4.3. Đồng bộ hóa luồng truyền (Thread-Safe TX with Mutex)
*   **Vấn đề:** Hàm `USART1_SendString` sử dụng cơ chế Polling vòng lặp cơ bản . Nếu `Task_DataSend` đang in chuỗi log lên ESP32 mà `Task_CommandProcess` cũng gọi hàm này để gửi phản hồi (ACK), hai chuỗi ký tự sẽ bị đan xen vào nhau, làm hỏng định dạng JSON .
*   **Giải pháp:** Xây dựng hàm bọc `Safe_UART_Send()` . Bất kỳ Task nào muốn gửi dữ liệu đều phải xin được "chìa khóa" Mutex (`xUartTxMutex`) . Nếu đường truyền đang bận, Task tới sau sẽ tự động vào trạng thái Blocked (chờ 100ms) cho đến khi đường truyền rảnh . Điều này giúp triệt tiêu hoàn toàn hiện tượng rác dữ liệu trên bus UART .
