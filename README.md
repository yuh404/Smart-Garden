<img width="348" height="435" alt="image" src="https://github.com/user-attachments/assets/fd1a1b2a-a4b0-4f98-a35d-a64699e3799c" />1. Project Overview (Tổng quan dự án)
Dự án này tập trung nghiên cứu, thiết kế và phát triển một nền tảng robot di động tự hành (Autonomous Mobile Robot - AMR) hoàn chỉnh, từ cấp độ vi mạch phần cứng đến hệ thống phần mềm điều khiển trung tâm. Mục tiêu cốt lõi là tạo ra một khung gầm tự hành linh hoạt, có độ tin cậy cao và sẵn sàng tích hợp các thuật toán điều hướng thông minh.
![Uploading image.png…]()


Đặc điểm nổi bật nhất của dự án là việc ứng dụng và hiện thực hóa mô hình Hệ thống Không gian mạng - Thực thể (Cyber-Physical System - CPS). Đây là sự giao thoa khăng khít và đồng bộ giữa hai luồng xử lý:
Không gian mạng (Cyber - Tính toán & Ra quyết định): Lớp xử lý cấp cao được đại diện bởi mạng lưới Robot Operating System (ROS 2). Đây là nơi đảm nhiệm việc tiếp nhận dữ liệu không gian, chạy các thuật toán hoạch định quỹ đạo (Navigation/SLAM) và phát lệnh điều hướng vận tốc (cmd_vel) xuống phần cứng.

Thực thể vật lý (Physical - Tương tác với môi trường thực): Lớp điều khiển cơ điện thời gian thực (đảm nhiệm bởi vi điều khiển STM32 kết hợp hệ điều hành FreeRTOS). Lớp này chịu trách nhiệm thu thập, đo lường các đại lượng vật lý thực tế từ cảm biến (vận tốc Encoder, góc nghiêng IMU) và bám sát mục tiêu thông qua việc điều khiển các cơ cấu chấp hành (Động cơ DC/BLDC) với độ trễ tính bằng mili-giây.



2. System Architecture (Kiến trúc Hệ thống)
Hệ thống được thiết kế theo cấu trúc Master-Slave, phân tách rõ ràng giữa lớp xử lý tính toán cấp cao (ROS 2) và lớp điều khiển cơ điện thời gian thực. 



2.1. Lớp điều khiển cấp thấp (Low-level Controller - STM32)
Đóng vai trò là bộ não điều khiển chuyển động trực tiếp, STM32 xử lý toàn bộ các vòng lặp phản hồi tín hiệu từ cảm biến và động cơ: 

Điều khiển động cơ (Motor Control & PWM): STM32 sử dụng các bộ Hardware Timer để xuất xung PWM điều khiển động cơ DC và BLDC. Hàm điều khiển can thiệp trực tiếp vào thanh ghi (__HAL_TIM_SET_COMPARE) để thay đổi độ rộng xung, hỗ trợ cấu hình linh hoạt cả chế độ băm xung thuận (Normal) và nghịch (Inverse) cho các mạch cầu H. 


Đọc phản hồi vị trí (Encoder Feedback): Tín hiệu Encoder được đọc trực tiếp từ thanh ghi đếm của Timer (__HAL_TIM_GET_COUNTER), hỗ trợ đọc nhiều chế độ như X1, X4 và quy đổi góc. Đặc biệt, vận tốc thực tế sau khi tính toán sẽ được đưa qua một thuật toán lọc thông thấp (Low-pass Filter) dựa trên dữ liệu quá khứ để loại bỏ nhiễu trước khi đưa vào vòng lặp điều khiển. 


Vòng lặp điều khiển kín (Closed-loop PID): Hệ thống sử dụng bộ điều khiển PID tùy chỉnh để bám sát và ổn định vận tốc mục tiêu. Thuật toán PID được thiết kế chuyên sâu với bộ lọc cho khâu vi phân (Derivative Filter), cơ chế bão hòa ngõ ra (Saturation bounds) để chống hiện tượng Windup, và hàm Reset chủ động xóa bộ nhớ tích phân khi cần thiết. 


Cảm biến quán tính (IMU MPU6050): STM32 giao tiếp với cảm biến MPU6050 qua chuẩn I2C để thu thập gia tốc và vận tốc góc. Thuật toán lọc Kalman (Kalman Filter) được lập trình bằng C và nhúng trực tiếp trên vi điều khiển để dung hợp dữ liệu, giúp triệt tiêu nhiễu và tính toán góc nghiêng (Roll, Pitch) một cách chính xác nhất. 

2.2. Lớp cổng kết nối cấp cao (High-level Gateway - ESP32)
ESP32 hoạt động như một Gateway IoT, kết nối phần cứng vi điều khiển với mạng lưới Robot Operating System (ROS 2): 

Giao tiếp ROS 2 qua Wi-Fi: ESP32 sử dụng thư viện micro_ros_arduino để khởi tạo một Node ROS 2 độc lập (tên node: esp32_agv_node), kết nối trực tiếp với máy tính chủ (Agent) thông qua mạng Wi-Fi. 

Định tuyến luồng lệnh (cmd_vel & Heartbeat): ESP32 đăng ký (Subscribe) topic cmd_vel để nhận vận tốc tuyến tính (vx) và vận tốc góc (wz) từ hệ thống điều hướng ROS. Các giá trị này lập tức được phân tách và đóng gói thành chuỗi định dạng (vx;wz) để truyền xuống STM32 qua Hardware UART2 (chân 16, 17). Hệ thống tích hợp sẵn cơ chế Heartbeat, tự động gửi lặp lại lệnh điều khiển mỗi 100ms để đảm bảo robot không bị lỡ tín hiệu hoặc mất kiểm soát nếu rớt mạng. 

Đồng bộ dữ liệu cục bộ (Odometry): Ở chiều ngược lại, ESP32 liên tục lắng nghe luồng dữ liệu từ STM32 gửi lên. Khi nhận diện được bản tin bắt đầu bằng ODOM, ESP32 sẽ tách (parse) tọa độ X, Y và góc định hướng Theta, sau đó đưa vào cấu trúc tin nhắn chuẩn nav_msgs/Odometry và xuất bản (Publish) lên topic odom để phục vụ các thuật toán SLAM/Navigation của ROS 2. 

3. Software & RTOS Implementation (Kiến trúc Phần mềm)
Hệ thống điều khiển trung tâm (STM32) được xây dựng trên nền tảng hệ điều hành thời gian thực FreeRTOS, đảm bảo tính định thời khắt khe (deterministic timing) cho các vòng lặp điều khiển và xử lý cảm biến. 

3.1. FreeRTOS Core Configuration
Timebase Isolation (Cách ly bộ định thời): Để tránh xung đột hệ thống, bộ đếm SysTick mặc định được nhường toàn quyền cho bộ lập lịch của FreeRTOS quản lý. Các hàm API trễ của phần cứng (HAL Delay) được cấu hình chạy độc lập trên Hardware Timer 3 (TIM3). 

Tick Rate & Scheduling: Hệ điều hành được thiết lập tần số trích mẫu 1000Hz (độ phân giải 1ms) và sử dụng cơ chế ưu tiên ngắt Preemptive Scheduling (configUSE_PREEMPTION = 1). Nhờ đó, các tác vụ tính toán động cơ quan trọng luôn được ưu tiên xử lý đúng hạn. 

3.2. Quản lý Đa nhiệm (Task Management)
Luồng thực thi chính của robot được chia thành 2 Task độc lập với mức độ ưu tiên và chu kỳ thời gian thực khác nhau: 

ControlTask (Priority: High - Chu kỳ 20ms): Đây là vòng lặp cốt lõi của robot, thực thi các tính toán toán học nặng nề một cách tuần tự. 

Đọc dữ liệu La bàn quán tính (MPU6050) và trích xuất số xung từ hai Encoder. 

Giải mã động học (Odometry) để cập nhật tọa độ tuyệt đối (X, Y) và góc định hướng (Theta) của robot. 

Heading Assist: Tính năng hỗ trợ giữ thẳng lái. Khi robot nhận lệnh chạy thẳng (không có vận tốc góc), hệ thống tự động khóa góc mục tiêu (Target Yaw) và sử dụng dữ liệu từ MPU6050 để xuất lệnh bù trừ PID, chống lại hiện tượng trượt bánh. 

Tính toán vận tốc mục tiêu cho từng bánh và đẩy qua bộ điều khiển PID trước khi xuất xung PWM xuống mạch công suất. 

CommTask (Priority: Normal - Chu kỳ 50ms): Tác vụ giao tiếp với hệ thống ROS 2 cấp cao. Task này có nhiệm vụ đóng gói các thông số tọa độ hiện tại thành chuỗi định dạng ODOM|X|Y|Theta\n và xuất lên ESP32 thông qua UART ngắt không đồng bộ (Non-blocking IT) với tần số 20Hz. 

3.3. Xử lý Truyền nhận và Toàn vẹn Dữ liệu
Việc giao tiếp UART giữa STM32 và ESP32 là điểm nghẽn tiềm ẩn nếu sử dụng vòng lặp chờ. Dự án giải quyết bằng các kỹ thuật sau: 

Bộ đệm vòng DMA & IDLE Line Detection: Luồng nhận lệnh cmd_vel từ ESP32 được cấu hình sử dụng DMA dạng Circular (DMA_CIRCULAR) kết hợp với ngắt báo rảnh đường truyền (HAL_UARTEx_ReceiveToIdle_DMA). Kỹ thuật này giúp CPU không bị treo khi chờ từng byte dữ liệu, tự động gom đủ một gói tin hoàn chỉnh trước khi kích hoạt hàm phân tích chuỗi. 

Bẫy lỗi đường truyền (UART Error Recovery): Trong môi trường nhiễu điện từ từ động cơ, đường truyền UART có thể bị kẹt hoặc tràn bộ đệm (Overrun). Hệ thống tích hợp sẵn hàm bẫy lỗi (HAL_UART_ErrorCallback). Ngay khi phát hiện sự cố, tiến trình DMA sẽ bị hủy (Abort) và khởi động lại luồng nhận mới, đảm bảo mạch không bị treo cứng. 

Watchdog Cơ chế an toàn (ROS 2 Fail-safe): Robot liên tục giám sát thời gian nhận lệnh cuối cùng. Nếu sau 2000ms (2 giây) không có tín hiệu điều hướng mới từ mạng Wi-Fi hoặc ESP32, toàn bộ thông số vận tốc lập tức bị ép về 0.0f để phanh khẩn cấp, ngăn chặn hiện tượng robot mất kiểm soát và đâm va. 

4. Communication Protocol & Data Integrity (Giao thức Truyền thông & Tính toàn vẹn dữ liệu)
Điểm nhấn Kỹ thuật (Technical Highlight): Đối mặt với thách thức truyền tín hiệu tốc độ cao (Baudrate 115200) trong môi trường có nhiễu điện từ (EMI) mạnh sinh ra từ các động cơ DC, dự án không sử dụng các vòng lặp chờ ngắt truyền thống. Thay vào đó, hệ thống áp dụng một kiến trúc truyền thông bất đồng bộ kết hợp các cơ chế bảo vệ phần mềm nghiêm ngặt. 

4.1. Lightweight Text Protocol (Giao thức văn bản tối ưu)
Thay vì sử dụng các khung truyền nhị phân (Binary Framing) cồng kềnh, dự án tự định nghĩa một giao thức dạng chuỗi cực kỳ nhẹ và trực quan, giúp tối ưu hóa bộ nhớ cho vi điều khiển và dễ dàng Debug: 

Luồng Downlink (ESP32 -> STM32): Bản tin vận tốc được bọc trong cấu trúc dấu ngoặc (vx;wz). 

Luồng Uplink (STM32 -> ESP32): Dữ liệu động học được đóng gói thành ODOM|X|Y|Theta\n. 

4.2. Bảo vệ toàn vẹn dữ liệu bằng Parsing Boundary (Biên định dạng)
Thay vì tốn tài nguyên CPU để tính toán mã CRC cho từng gói tin, hệ thống đảm bảo tính chính xác của dữ liệu bằng thuật toán dò tìm biên giới hạn (Boundary Parsing): 

Khi có luồng chuỗi đi vào, STM32 sử dụng các con trỏ quét qua bộ đệm để định vị chính xác vị trí của ký tự mở ngoặc (, dấu phân cách ; và đóng ngoặc ). 

Chống sai lệch: Nếu nhiễu từ động cơ làm đứt gãy hoặc mất byte trên đường truyền, các dấu hiệu nhận biết này sẽ bị thiếu, con trỏ trả về NULL. Gói tin lỗi sẽ ngay lập tức bị loại bỏ trước khi hàm atof() kịp dịch ra số, ngăn chặn tuyệt đối tình trạng robot chạy loạn do nhận sai vận tốc. 

4.3. DMA Idle Detection & Error Recovery (Chống nghẽn & Tự phục hồi)
DMA với Idle Line: STM32 sử dụng tính năng HAL_UARTEx_ReceiveToIdle_DMA để thu thập dữ liệu. Chíp tự động gom các byte nhận được vào bộ đệm vòng (Circular Buffer) dưới nền phần cứng mà không làm phiền CPU. Chỉ khi đường truyền dứt (trạng thái IDLE) báo hiệu đã gửi xong một gói lệnh, CPU mới được đánh thức để xử lý. 

Tự động phục hồi (Self-Recovery): Điểm sáng giá nhất của hệ thống là hàm bẫy lỗi HAL_UART_ErrorCallback. Khi tần số gửi lệnh quá cao làm tràn thanh ghi cứng (Overrun Error) – nguyên nhân chính gây "treo" vi điều khiển, hệ thống sẽ bắt lỗi, chủ động hủy luồng DMA kẹt (HAL_UART_AbortReceive) và ngay lập tức mở lại một luồng DMA mới để làm sạch bộ đệm. Nhờ đó, robot có khả năng tự phục hồi tức thì mà không cần khởi động lại. 

4.4. ROS 2 Watchdog & Heartbeat (Cơ chế an toàn chủ động)
ESP32 Heartbeat: Ngay cả khi không có lệnh vận tốc mới, ESP32 vẫn định kỳ (mỗi 100ms) bơm lại gói lệnh cũ xuống để giữ nhịp đường truyền. 

STM32 Fail-safe: STM32 liên tục giám sát biến thời gian last_cmd_time. Nếu quá 2000ms mà không nhận được bất kỳ chuỗi lệnh hợp lệ nào (do đứt cáp, rớt mạng Wi-Fi, máy tính ROS 2 sập), hệ thống Watchdog nội bộ sẽ kích hoạt, tự động cắt toàn bộ xung PWM và ép vận tốc về 0.0f để phanh khẩn cấp, đảm bảo an toàn tuyệt đối cho thiết bị và môi trường xung quanh. 

5. Hardware Setup & Pinout (Sơ đồ kết nối)
Hệ thống được kết nối dựa trên vi điều khiển trung tâm STM32F103C8T6 (Blue Pill) và module giao tiếp ESP32. Dưới đây là sơ đồ nối dây chi tiết cho các ngoại vi quan trọng

5.1. STM32 MCU Pinout (Low-level Controller)
Điều khiển Động cơ (Motor Driver & PWM): Sử dụng Timer 2 (TIM2) ở chế độ PWM. 

PA0 (TIM2_CH1): PWM Kênh 1 - Động cơ Phải (Right Motor). 

PA1 (TIM2_CH2): PWM Kênh 2 - Động cơ Phải (Right Motor). 

PA2 (TIM2_CH3): PWM Kênh 3 - Động cơ Trái (Left Motor). 

PA3 (TIM2_CH4): PWM Kênh 4 - Động cơ Trái (Left Motor). 

Đọc phản hồi Encoder (External Interrupt): Sử dụng ngắt ngoài (EXTI9_5) kết hợp vòng lặp kiểm tra trạng thái kênh B để xác định chiều quay. 

GPIOB (Chân ngắt kênh A - Động cơ Trái): en_A_L_Pin. 

GPIOB (Chân đọc kênh B - Động cơ Trái): en_B_L_Pin. 

GPIOB (Chân ngắt kênh A - Động cơ Phải): en_A_R_Pin. 

GPIOB (Chân đọc kênh B - Động cơ Phải): en_B_R_Pin. 

Cảm biến quán tính MPU6050 (I2C1):

PB8: I2C1_SCL (Xung nhịp đồng bộ). 

PB9: I2C1_SDA (Đường truyền dữ liệu). 

5.2. UART Communication Bridge (Cầu nối STM32 - ESP32)
Để đảm bảo tốc độ truyền tải 115200 bps mà không bị nhiễu chéo, hai vi điều khiển được kết nối chéo chân (TX nối RX) thông qua giao diện Hardware Serial: 
STM32 (USART1)	ESP32 (Hardware Serial 2)	Chức năng (Định tuyến luồng)
PA9 (TX)	GPIO 16 (RX2)	Gửi dữ liệu Odometry (X, Y, Theta) lên ROS 2. 
PA10 (RX)	GPIO 17 (TX2)	Nhận lệnh Vận tốc (vx, wz) từ ROS 2 đẩy xuống. 
GND	GND	Bắt buộc: Nối chung mass (GND) để đồng bộ mức điện áp chuẩn. 
