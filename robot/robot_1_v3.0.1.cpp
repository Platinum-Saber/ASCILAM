// Arduino Motor Controller with Encoders and MPU6050
// Receives commands from ESP32 and provides odometry feedback
// Robot ID: Change this for each robot
// Set to 1 for Robot 1, 2 for Robot 2

#include <Wire.h>
#include <MPU6050_light.h>

const int ROBOT_ID = 1; 

// Motor Control Pins (L298N)
#define ENA 5   // Motor A PWM (LEFT)
#define IN1 8   // Motor A Direction 1
#define IN2 9   // Motor A Direction 2
#define ENB 6   // Motor B PWM (RIGHT)
#define IN3 10  // Motor B Direction 1
#define IN4 11  // Motor B Direction 2

// Encoder Pins
#define ENCODER_A_PIN 2  // Left encoder (interrupt pin)
#define ENCODER_B_PIN 3  // Right encoder (interrupt pin)

// Robot Physical Parameters
#define WHEEL_DIAMETER 0.065      // 65mm wheels in meters
#define WHEEL_BASE 0.15           // 150mm between wheels in meters
#define ENCODER_PULSES_PER_REV 20 // Pulses per revolution
#define GEAR_RATIO 1.0            // Adjust if using geared motors

// Calculated constants
const float METERS_PER_PULSE = (PI * WHEEL_DIAMETER) / (ENCODER_PULSES_PER_REV * GEAR_RATIO);

// Encoder counters
volatile long encoder_left_count = 0;
volatile long encoder_right_count = 0;

// Odometry variables
float robot_x = 0.0;
float robot_y = 0.0;
float robot_theta = 0.0;
unsigned long last_odom_time = 0;

// MPU6050
MPU6050 mpu(Wire);
float gyro_offset_z = 0.0;
bool mpu_initialized = false;

// Motor control
int motor_left_speed = 0;
int motor_right_speed = 0;
unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT = 500; // Stop motors if no command for 500ms

// PID for motor control (optional enhancement)
float target_left_speed = 0.0;
float target_right_speed = 0.0;

void setup() {
    // Initialize serial communication with ESP32
    Serial.begin(9600);
    
    // Initialize motor pins
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    // Initialize encoder pins
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);
    
    // Attach interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoder_left_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoder_right_ISR, RISING);
    
    // Initialize MPU6050
    Wire.begin();
    byte status = mpu.begin();
    if (status == 0) {
        mpu.calcOffsets(true, true); // Gyro and accelerometer
        gyro_offset_z = mpu.getAngleZ();
        mpu_initialized = true;
        Serial.println("STATUS:MPU6050_OK");
    } else {
        Serial.print("STATUS:MPU6050_ERROR:");
        Serial.println(status);
    }
    
    // Send ready message
    Serial.print("READY:Robot");
    Serial.println(ROBOT_ID);
    
    // Stop motors initially
    stop_motors();
    
    last_odom_time = millis();
    last_cmd_time = millis();
}

void loop() {
    unsigned long current_time = millis();
    
    // Check for commands from ESP32
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command.length() > 0) {
            execute_command(command);
            last_cmd_time = current_time;
        }
    }
    
    // Safety: Stop motors if no command received
    if (current_time - last_cmd_time > CMD_TIMEOUT) {
        if (motor_left_speed != 0 || motor_right_speed != 0) {
            stop_motors();
            Serial.println("STATUS:TIMEOUT_STOP");
        }
    }
    
    // Update MPU6050
    if (mpu_initialized) {
        mpu.update();
    }
    
    // Update and send odometry at 20Hz
    if (current_time - last_odom_time >= 50) {
        update_odometry();
        send_odometry();
        last_odom_time = current_time;
    }
    
    delay(10);
}

// Encoder interrupt service routines
void encoder_left_ISR() {
    // Increment or decrement based on motor direction
    if (motor_left_speed >= 0) {
        encoder_left_count++;
    } else {
        encoder_left_count--;
    }
}

void encoder_right_ISR() {
    // Increment or decrement based on motor direction
    if (motor_right_speed >= 0) {
        encoder_right_count++;
    } else {
        encoder_right_count--;
    }
}

void execute_command(String cmd) {
    if (cmd.length() == 0) return;
    
    char command_type = cmd.charAt(0);
    float value = 0;
    
    if (cmd.length() > 1) {
        value = cmd.substring(1).toFloat();
    }
    
    switch (command_type) {
        case 'M': // Move command (PWM value)
            if (value > 0) {
                move_forward(abs(value));
            } else if (value < 0) {
                move_backward(abs(value));
            } else {
                stop_motors();
            }
            break;
            
        case 'T': // Turn command (angular velocity in degrees/sec)
            if (value > 0) {
                turn_right(abs(value));
            } else if (value < 0) {
                turn_left(abs(value));
            }
            break;
            
        case 'D': // Differential drive command: D<left_pwm>,<right_pwm>
            {
                int comma_pos = cmd.indexOf(',', 1);
                if (comma_pos > 0) {
                    int left_pwm = cmd.substring(1, comma_pos).toInt();
                    int right_pwm = cmd.substring(comma_pos + 1).toInt();
                    set_motor_speeds(left_pwm, right_pwm);
                }
            }
            break;
            
        case 'S': // Stop command
            stop_motors();
            break;
            
        case 'R': // Reset odometry
            reset_odometry();
            Serial.println("STATUS:ODOM_RESET");
            break;
            
        default:
            stop_motors();
            break;
    }
}

void move_forward(float speed) {
    int motor_pwm = constrain(speed, 0, 255);
    set_motor_speeds(motor_pwm, motor_pwm);
}

void move_backward(float speed) {
    int motor_pwm = constrain(speed, 0, 255);
    set_motor_speeds(-motor_pwm, -motor_pwm);
}

void turn_left(float angular_speed) {
    // Convert angular speed to differential wheel speeds
    int turn_speed = map(angular_speed, 0, 360, 0, 150);
    turn_speed = constrain(turn_speed, 0, 150);
    set_motor_speeds(-turn_speed, turn_speed);
}

void turn_right(float angular_speed) {
    // Convert angular speed to differential wheel speeds
    int turn_speed = map(angular_speed, 0, 360, 0, 150);
    turn_speed = constrain(turn_speed, 0, 150);
    set_motor_speeds(turn_speed, -turn_speed);
}

void set_motor_speeds(int left_pwm, int right_pwm) {
    // Left motor (Motor A)
    motor_left_speed = constrain(left_pwm, -255, 255);
    if (motor_left_speed > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, motor_left_speed);
    } else if (motor_left_speed < 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, abs(motor_left_speed));
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 0);
    }
    
    // Right motor (Motor B)
    motor_right_speed = constrain(right_pwm, -255, 255);
    if (motor_right_speed > 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, motor_right_speed);
    } else if (motor_right_speed < 0) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, abs(motor_right_speed));
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, 0);
    }
}

void stop_motors() {
    motor_left_speed = 0;
    motor_right_speed = 0;
    
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
    
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
}

void update_odometry() {
    static long last_left_count = 0;
    static long last_right_count = 0;
    static unsigned long last_update_time = 0;
    
    unsigned long current_time = millis();
    float dt = (current_time - last_update_time) / 1000.0;
    
    if (dt <= 0) {
        last_update_time = current_time;
        return;
    }
    
    // Get encoder counts
    noInterrupts();
    long left_count = encoder_left_count;
    long right_count = encoder_right_count;
    interrupts();
    
    // Calculate distance traveled by each wheel
    long delta_left = left_count - last_left_count;
    long delta_right = right_count - last_right_count;
    
    float distance_left = delta_left * METERS_PER_PULSE;
    float distance_right = delta_right * METERS_PER_PULSE;
    
    // Calculate robot displacement
    float distance_center = (distance_left + distance_right) / 2.0;
    float delta_theta = (distance_right - distance_left) / WHEEL_BASE;
    
    // Update pose using dead reckoning
    float delta_x = distance_center * cos(robot_theta + delta_theta / 2.0);
    float delta_y = distance_center * sin(robot_theta + delta_theta / 2.0);
    
    robot_x += delta_x;
    robot_y += delta_y;
    
    // Fuse with MPU6050 gyro for better heading
    if (mpu_initialized) {
        float gyro_angle = (mpu.getAngleZ() - gyro_offset_z) * DEG_TO_RAD;
        // Use complementary filter (80% encoder, 20% gyro)
        robot_theta = 0.8 * (robot_theta + delta_theta) + 0.2 * gyro_angle;
    } else {
        robot_theta += delta_theta;
    }
    
    // Normalize theta to [-PI, PI]
    while (robot_theta > PI) robot_theta -= 2 * PI;
    while (robot_theta < -PI) robot_theta += 2 * PI;
    
    // Update last values
    last_left_count = left_count;
    last_right_count = right_count;
    last_update_time = current_time;
}

void send_odometry() {
    // Calculate velocities
    static float last_x = 0;
    static float last_y = 0;
    static float last_theta = 0;
    static unsigned long last_send_time = 0;
    
    unsigned long current_time = millis();
    float dt = (current_time - last_send_time) / 1000.0;
    
    if (dt > 0) {
        float vx = (robot_x - last_x) / dt;
        float vy = (robot_y - last_y) / dt;
        float vtheta = (robot_theta - last_theta) / dt;
        
        // Send odometry data to ESP32
        // Format: ODOM:x,y,theta,vx,vy,vtheta
        Serial.print("ODOM:");
        Serial.print(robot_x, 4);
        Serial.print(",");
        Serial.print(robot_y, 4);
        Serial.print(",");
        Serial.print(robot_theta, 4);
        Serial.print(",");
        Serial.print(vx, 4);
        Serial.print(",");
        Serial.print(vy, 4);
        Serial.print(",");
        Serial.println(vtheta, 4);
        
        last_x = robot_x;
        last_y = robot_y;
        last_theta = last_theta;
        last_send_time = current_time;
    }
}

void reset_odometry() {
    noInterrupts();
    encoder_left_count = 0;
    encoder_right_count = 0;
    interrupts();
    
    robot_x = 0.0;
    robot_y = 0.0;
    robot_theta = 0.0;
    
    if (mpu_initialized) {
        gyro_offset_z = mpu.getAngleZ();
    }
}