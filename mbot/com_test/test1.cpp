// Simple M-Bot Debug Code - Just Print Commands
// Upload this to M-Bot to see what commands it receives from ESP32

void setup() {
    // Initialize serial communication with ESP32
    Serial.begin(9600);
    
    // Initialize built-in LED for visual feedback
    pinMode(13, OUTPUT);
    
    Serial.println("=== M-Bot Command Debug ===");
    Serial.println("Robot ID: 1");
    Serial.println("Listening for ESP32 commands...");
    Serial.println("============================");
    
    // Flash LED to indicate ready
    for(int i = 0; i < 3; i++) {
        digitalWrite(13, HIGH);
        delay(300);
        digitalWrite(13, LOW);
        delay(300);
    }
    
    Serial.println("READY: Waiting for commands");
}

void loop() {
    // Check if any data is available from ESP32
    if (Serial.available()) {
        // Turn on LED when receiving data
        digitalWrite(13, HIGH);
        
        // Read the command
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        // Print timestamp
        Serial.print("[");
        Serial.print(millis());
        Serial.print("ms] ");
        
        // Print the raw command
        Serial.print("RECEIVED: [");
        Serial.print(command);
        Serial.print("] ");
        
        // Print command length
        Serial.print("(Length: ");
        Serial.print(command.length());
        Serial.print(") ");
        
        // Parse and display command details
        if(command.length() > 0) {
            char cmdType = command.charAt(0);
            float cmdValue = 0;
            
            if(command.length() > 1) {
                cmdValue = command.substring(1).toFloat();
            }
            
            Serial.print("Type: ");
            Serial.print(cmdType);
            Serial.print(", Value: ");
            Serial.print(cmdValue);
            
            // Interpret command meaning
            Serial.print(" -> ");
            switch(cmdType) {
                case 'M':
                    if(cmdValue > 0) {
                        Serial.print("MOVE FORWARD (speed: ");
                        Serial.print(cmdValue);
                        Serial.print(")");
                    } else if(cmdValue < 0) {
                        Serial.print("MOVE BACKWARD (speed: ");
                        Serial.print(abs(cmdValue));
                        Serial.print(")");
                    } else {
                        Serial.print("MOVE (no speed)");
                    }
                    break;
                case 'T':
                    if(cmdValue > 0) {
                        Serial.print("TURN RIGHT (");
                        Serial.print(cmdValue);
                        Serial.print(" degrees)");
                    } else if(cmdValue < 0) {
                        Serial.print("TURN LEFT (");
                        Serial.print(abs(cmdValue));
                        Serial.print(" degrees)");
                    } else {
                        Serial.print("TURN (no angle)");
                    }
                    break;
                case 'S':
                    Serial.print("STOP");
                    break;
                case 'A':
                    Serial.print("SET AUTONOMOUS MODE");
                    break;
                case 'E':
                    Serial.print("EMERGENCY STOP");
                    break;
                default:
                    Serial.print("UNKNOWN COMMAND");
                    break;
            }
        } else {
            Serial.print("EMPTY COMMAND");
        }
        
        Serial.println();
        
        // Turn off LED
        digitalWrite(13, LOW);
        
        // Send acknowledgment back to ESP32
        Serial.print("ACK:");
        Serial.println(command);
    }
    
    // Send periodic heartbeat to show M-Bot is alive
    static unsigned long lastHeartbeat = 0;
    if(millis() - lastHeartbeat > 10000) { // Every 10 seconds
        Serial.print("HEARTBEAT: M-Bot alive at ");
        Serial.print(millis());
        Serial.println("ms");
        lastHeartbeat = millis();
    }
    
    delay(10);
}