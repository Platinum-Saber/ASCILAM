// ESP32: Robot2_microROS_v3.0_Arduino.cpp
// Updated for Arduino motor controller with encoders and MPU6050
// Robot 2 with LD19 LiDAR and Arduino integration

#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rmw_microros/rmw_microros.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// WiFi Configuration
char ssid[] = "MultiRobot_Network";
char password[] = "multirobot2024";
char agent_ip[] = "192.168.4.1";
const int agent_port = 8889;

// Hardware Configuration
#define LIDAR_SERIAL Serial2
#define LIDAR_BAUDRATE 230400
#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 17

HardwareSerial ArduinoSerial(1);
#define ARDUINO_RX_PIN 3
#define ARDUINO_TX_PIN 1

#define STATUS_LED 2

// LiDAR Configuration
#define POINT_PER_PACK 12
#define HEADER 0x54
#define OUT_BEAMS 360

// LiDAR packet structure
typedef struct __attribute__((packed)) {
    uint16_t distance;
    uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
    uint8_t header;
    uint8_t ver_len;
    uint16_t speed;
    uint16_t start_angle;
    LidarPointStructDef point[POINT_PER_PACK];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t crc8;
} LiDARFrameTypeDef;

// micro-ROS entities
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_timer_t timer;
rcl_publisher_t scan_publisher;
rcl_publisher_t odom_publisher;
rcl_subscription_t cmd_vel_subscriber;
rclc_executor_t executor;

// Messages
sensor_msgs__msg__LaserScan scan_msg;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

// LiDAR data
float ranges[OUT_BEAMS];
float intensities[OUT_BEAMS];
bool new_scan_ready = false;

// Robot state from Arduino odometry
float robot_x = 0.0;
float robot_y = 0.0;
float robot_theta = 0.0;
float robot_vx = 0.0;
float robot_vy = 0.0;
float robot_vtheta = 0.0;
bool odom_data_valid = false;

// Last command tracking
float last_cmd_linear = 0.0;
float last_cmd_angular = 0.0;
unsigned long last_cmd_time = 0;
unsigned long last_odom_received = 0;

// Connection status
bool wifi_connected = false;
bool micro_ros_connected = false;
bool arduino_connected = false;
unsigned long last_wifi_check = 0;
unsigned long last_status_blink = 0;
unsigned long last_arduino_check = 0;
bool status_led_state = false;

// Robot parameters
const float wheel_base = 0.15;
const float max_speed = 0.5;

// CRC table from LD19 manual
static const uint8_t CrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

uint8_t CalCRC8(uint8_t *p, uint8_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc = CrcTable[(crc ^ *p++) & 0xff];
    }
    return crc;
}

// Error handling macros
#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)){ \
        Serial.printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); \
        error_loop(); \
    } \
}

#define RCSOFTCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)){ \
        Serial.printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc); \
    } \
}

void error_loop() {
    while(true) {
        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
        delay(100);
    }
}

// Callback functions
void cmd_vel_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    
    last_cmd_linear = msg->linear.x;
    last_cmd_angular = msg->angular.z;
    last_cmd_time = millis();
    
    // DEBUG: Print received command
    Serial.print("ROS CMD - Linear: ");
    Serial.print(msg->linear.x, 3);
    Serial.print(", Angular: ");
    Serial.println(msg->angular.z, 3);
    
    // Convert to differential drive commands
    float linear = msg->linear.x;
    float angular = msg->angular.z;
    
    // Calculate wheel speeds
    float left_speed = linear - (angular * wheel_base / 2.0);
    float right_speed = linear + (angular * wheel_base / 2.0);
    
    // Convert to PWM values (-255 to 255)
    int left_pwm = constrain(left_speed * 255 / max_speed, -255, 255);
    int right_pwm = constrain(right_speed * 255 / max_speed, -255, 255);
    
    // Send differential drive command to Arduino
    if (abs(left_pwm) < 10 && abs(right_pwm) < 10) {
        Serial.println("Sending: S0");
        ArduinoSerial.println("S0");
    } else {
        Serial.print("Sending: D");
        Serial.print(left_pwm);
        Serial.print(",");
        Serial.println(right_pwm);
        
        ArduinoSerial.print("D");
        ArduinoSerial.print(left_pwm);
        ArduinoSerial.print(",");
        ArduinoSerial.println(right_pwm);
    }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL && micro_ros_connected) {
        if (new_scan_ready) {
            publish_scan();
            new_scan_ready = false;
        }
        if (odom_data_valid) {
            publish_odometry();
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, HIGH);
    
    // Initialize hardware
    LIDAR_SERIAL.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
    ArduinoSerial.begin(9600, SERIAL_8N1, ARDUINO_RX_PIN, ARDUINO_TX_PIN);
    
    // Initialize data arrays
    for (int i = 0; i < OUT_BEAMS; i++) {
        ranges[i] = 12.0;
        intensities[i] = 0.0;
    }
    
    // Wait for Arduino to initialize
    delay(2000);
    
    // Connect to WiFi
    connect_wifi();
    
    // Setup micro-ROS
    if (wifi_connected) {
        setup_micro_ros();
    }
    
    Serial.println("Robot 2 initialized with Arduino motor controller");
}

void loop() {
    check_wifi_status();
    check_arduino_status();
    read_lidar_data();
    read_arduino_data();
    
    if (micro_ros_connected) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
    
    update_status_led();
    delay(1);
}

void connect_wifi() {
    Serial.print("Robot 2 connecting to WiFi: ");
    Serial.println(ssid);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    WiFi.setHostname("robot2");
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = true;
        Serial.println();
        Serial.print("Robot 2 WiFi connected! IP: ");
        Serial.println(WiFi.localIP());
    } else {
        wifi_connected = false;
        Serial.println();
        Serial.println("Robot 2 WiFi connection failed!");
    }
}

void check_wifi_status() {
    unsigned long now = millis();
    if (now - last_wifi_check > 10000) {
        if (WiFi.status() != WL_CONNECTED && wifi_connected) {
            wifi_connected = false;
            micro_ros_connected = false;
            Serial.println("Robot 2 WiFi disconnected! Reconnecting...");
            connect_wifi();
            if (wifi_connected) {
                setup_micro_ros();
            }
        }
        last_wifi_check = now;
    }
}

void check_arduino_status() {
    unsigned long now = millis();
    if (now - last_arduino_check > 5000) {
        if (now - last_odom_received > 2000 && arduino_connected) {
            arduino_connected = false;
            Serial.println("Warning: No data from Arduino!");
        }
        last_arduino_check = now;
    }
}

void setup_micro_ros() {
    if (!wifi_connected) return;
    
    set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
    
    Serial.print("Pinging micro-ROS agent");
    while (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
        Serial.print(".");
        delay(200);
    }
    Serial.println(" OK");
    
    rmw_uros_sync_session(1000);
    
    allocator = rcl_get_default_allocator();
    
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "robot2_node", "", &support));
    
    // Create publishers
    RCCHECK(rclc_publisher_init_default(
        &scan_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "/robot2/scan"));
    
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/robot2/odom"));
    
    // Create subscriber
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/robot2/cmd_vel"));
    
    // Create timer (10Hz)
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback));
    
    // Create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    
    // Initialize messages
    init_messages();
    
    micro_ros_connected = true;
    Serial.println("Robot 2 micro-ROS initialized");
}

void init_messages() {
    // Initialize scan message
    sensor_msgs__msg__LaserScan__init(&scan_msg);
    rosidl_runtime_c__String__init(&scan_msg.header.frame_id);
    rosidl_runtime_c__String__assign(&scan_msg.header.frame_id, "robot2/lidar_link");
    
    // Allocate ranges and intensities arrays
    if (!rosidl_runtime_c__float__Sequence__init(&scan_msg.ranges, OUT_BEAMS)) {
        error_loop();
    }
    if (!rosidl_runtime_c__float__Sequence__init(&scan_msg.intensities, OUT_BEAMS)) {
        error_loop();
    }
    
    // Set scan parameters
    scan_msg.angle_min = 0.0;
    scan_msg.angle_max = 2.0 * M_PI;
    scan_msg.angle_increment = (2.0 * M_PI) / OUT_BEAMS;
    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = 0.1;
    scan_msg.range_min = 0.15;
    scan_msg.range_max = 12.0;
    
    // Initialize odometry message
    nav_msgs__msg__Odometry__init(&odom_msg);
    rosidl_runtime_c__String__init(&odom_msg.header.frame_id);
    rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "robot2/odom");
    rosidl_runtime_c__String__init(&odom_msg.child_frame_id);
    rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "robot2/base_link");
}

void read_lidar_data() {
    static uint8_t buffer[47];
    static int buffer_index = 0;
    static bool frame_started = false;
    
    while (LIDAR_SERIAL.available()) {
        uint8_t byte = LIDAR_SERIAL.read();
        
        if (!frame_started && byte == HEADER) {
            frame_started = true;
            buffer[0] = byte;
            buffer_index = 1;
        } else if (frame_started) {
            buffer[buffer_index++] = byte;
            
            if (buffer_index >= 47) {
                process_lidar_frame(buffer);
                frame_started = false;
                buffer_index = 0;
            }
        }
    }
}

void process_lidar_frame(uint8_t* buffer) {
    LiDARFrameTypeDef* frame = (LiDARFrameTypeDef*)buffer;
    
    if (frame->header != HEADER || frame->ver_len != 0x2C) return;
    
    uint8_t calculated_crc = CalCRC8(buffer, 46);
    if (calculated_crc != frame->crc8) return;
    
    float start_angle = frame->start_angle * 0.01 * M_PI / 180.0;
    float end_angle = frame->end_angle * 0.01 * M_PI / 180.0;
    
    if (end_angle < start_angle) end_angle += 2 * M_PI;
    
    float angle_step = (end_angle - start_angle) / (POINT_PER_PACK - 1);
    
    for (int i = 0; i < POINT_PER_PACK; i++) {
        float angle = start_angle + i * angle_step;
        if (angle >= 2 * M_PI) angle -= 2 * M_PI;
        
        int angle_index = (int)(angle * 180.0 / M_PI);
        if (angle_index >= 0 && angle_index < OUT_BEAMS) {
            float distance = frame->point[i].distance / 1000.0;
            uint8_t intensity = frame->point[i].intensity;
            
            if (distance >= 0.15 && distance <= 12.0) {
                ranges[angle_index] = distance;
                intensities[angle_index] = intensity;
            }
        }
    }
    
    new_scan_ready = true;
}

void read_arduino_data() {
    if (ArduinoSerial.available()) {
        String data = ArduinoSerial.readStringUntil('\n');
        data.trim();
        
        if (data.startsWith("ODOM:")) {
            // Parse odometry data: ODOM:x,y,theta,vx,vy,vtheta
            parse_odometry(data.substring(5));
            last_odom_received = millis();
            arduino_connected = true;
            odom_data_valid = true;
        } else if (data.startsWith("STATUS:")) {
            Serial.print("Arduino: ");
            Serial.println(data);
        } else if (data.startsWith("READY:")) {
            arduino_connected = true;
            Serial.print("Arduino ready: ");
            Serial.println(data);
        }
    }
}

void parse_odometry(String odom_data) {
    // Parse: x,y,theta,vx,vy,vtheta
    int idx = 0;
    int lastIdx = 0;
    float values[6];
    int count = 0;
    
    while (idx >= 0 && count < 6) {
        idx = odom_data.indexOf(',', lastIdx);
        if (idx >= 0) {
            values[count] = odom_data.substring(lastIdx, idx).toFloat();
            lastIdx = idx + 1;
        } else {
            values[count] = odom_data.substring(lastIdx).toFloat();
        }
        count++;
    }
    
    if (count == 6) {
        robot_x = values[0];
        robot_y = values[1];
        robot_theta = values[2];
        robot_vx = values[3];
        robot_vy = values[4];
        robot_vtheta = values[5];
    }
}

void publish_scan() {
    if (!micro_ros_connected) return;
    
    // Update timestamp
    uint64_t now_ns = rmw_uros_epoch_nanos();
    scan_msg.header.stamp.sec = (int32_t)(now_ns / 1000000000ULL);
    scan_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000ULL);
    
    // Copy ranges data
    for (int i = 0; i < OUT_BEAMS; i++) {
        scan_msg.ranges.data[i] = ranges[i];
        scan_msg.intensities.data[i] = intensities[i];
    }
    
    RCSOFTCHECK(rcl_publish(&scan_publisher, &scan_msg, NULL));
}

void publish_odometry() {
    if (!micro_ros_connected || !odom_data_valid) return;
    
    // Update timestamp
    uint64_t now_ns = rmw_uros_epoch_nanos();
    odom_msg.header.stamp.sec = (int32_t)(now_ns / 1000000000ULL);
    odom_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000ULL);
    
    // Update pose from Arduino odometry
    odom_msg.pose.pose.position.x = robot_x;
    odom_msg.pose.pose.position.y = robot_y;
    odom_msg.pose.pose.position.z = 0.0;
    
    // Convert theta to quaternion
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(robot_theta / 2.0);
    odom_msg.pose.pose.orientation.w = cos(robot_theta / 2.0);
    
    // Update velocity from Arduino
    odom_msg.twist.twist.linear.x = robot_vx;
    odom_msg.twist.twist.linear.y = robot_vy;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = robot_vtheta;
    
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

void update_status_led() {
    unsigned long now = millis();
    if (now - last_status_blink > 500) {
        status_led_state = !status_led_state;
        
        if (micro_ros_connected && arduino_connected) {
            digitalWrite(STATUS_LED, HIGH); // Solid on when fully connected
        } else if (wifi_connected) {
            digitalWrite(STATUS_LED, status_led_state); // Slow blink when WiFi only
        } else {
            digitalWrite(STATUS_LED, (now % 200) < 100); // Fast blink when disconnected
        }
        
        last_status_blink = now;
    }
}