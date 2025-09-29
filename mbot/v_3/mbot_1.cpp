// Simple M-Bot code with correct pin mapping
// Just receives commands from ESP32 and moves
// Robot ID: Change this for each robot
const int ROBOT_ID = 1; // Set to 1 for Robot 1, 2 for Robot 2

// M-Bot pin definitions (correct mapping)
#define PWM1        6   // Motor 1 PWM
#define DIR1        7   // Motor 1 Direction  
#define PWM2        5   // Motor 2 PWM
#define DIR2        4   // Motor 2 Direction
#define BUZZER      8   // Buzzer
#define RGB_LED    13   // WS2812 RGB LEDs

// Motor speeds
int motor_speed = 150;

void setup() {
    // Initialize serial communication with ESP32
    Serial.begin(9600);
    
    // Initialize motor pins
    pinMode(PWM1, OUTPUT);
    pinMode(DIR1, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    
    // Startup beep
    digitalWrite(BUZZER, HIGH);
    delay(200);
    digitalWrite(BUZZER, LOW);
    
    // Send ready message
    Serial.print("READY:Robot");
    Serial.println(ROBOT_ID);
    
    // Stop motors initially
    stop_motors();
}

void loop() {
    // Check for commands from ESP32
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        // Print received command
        Serial.print("RECEIVED: '");
        Serial.print(command);
        Serial.println("'");
        
        // Execute command
        execute_command(command);
    }
    
    delay(10); // Small delay
}

void execute_command(String cmd) {
    if (cmd.length() == 0) {
        Serial.println("EMPTY COMMAND");
        return;
    }
    
    char command_type = cmd.charAt(0);
    float value = 0;
    
    if (cmd.length() > 1) {
        value = cmd.substring(1).toFloat();
    }
    
    Serial.print("EXECUTING: Type=");
    Serial.print(command_type);
    Serial.print(" Value=");
    Serial.println(value);
    
    switch (command_type) {
        case 'M': // Move command
            if (value > 0) {
                move_forward(abs(value));
                Serial.println("ACTION: Moving forward");
            } else if (value < 0) {
                move_backward(abs(value));
                Serial.println("ACTION: Moving backward");
            } else {
                stop_motors();
                Serial.println("ACTION: Stopping");
            }
            break;
            
        case 'T': // Turn command
            if (value > 0) {
                turn_right();
                Serial.println("ACTION: Turning right");
            } else if (value < 0) {
                turn_left();
                Serial.println("ACTION: Turning left");
            }
            break;
            
        case 'S': // Stop command
            stop_motors();
            Serial.println("ACTION: Stopping");
            break;
            
        default:
            Serial.print("UNKNOWN COMMAND: ");
            Serial.println(command_type);
            stop_motors();
            break;
    }
}

void move_forward(float speed) {
    int motor_pwm = constrain(speed, 0, 255);
    
    // Motor 1 forward
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, motor_pwm);
    
    // Motor 2 forward  
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, motor_pwm);
    
    Serial.print("MOTORS: Forward at speed ");
    Serial.println(motor_pwm);
}

void move_backward(float speed) {
    int motor_pwm = constrain(speed, 0, 255);
    
    // Motor 1 backward
    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, motor_pwm);
    
    // Motor 2 backward
    digitalWrite(DIR2, LOW);
    analogWrite(PWM2, motor_pwm);
    
    Serial.print("MOTORS: Backward at speed ");
    Serial.println(motor_pwm);
}

void turn_left() {
    // Motor 1 backward, Motor 2 forward
    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, motor_speed);
    
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, motor_speed);
    
    Serial.println("MOTORS: Turning left");
    
    delay(300); // Turn for 300ms
    stop_motors();
}

void turn_right() {
    // Motor 1 forward, Motor 2 backward
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, motor_speed);
    
    digitalWrite(DIR2, LOW);
    analogWrite(PWM2, motor_speed);
    
    Serial.println("MOTORS: Turning right");
    
    delay(300); // Turn for 300ms
    stop_motors();
}

void stop_motors() {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    Serial.println("MOTORS: Stopped");
}