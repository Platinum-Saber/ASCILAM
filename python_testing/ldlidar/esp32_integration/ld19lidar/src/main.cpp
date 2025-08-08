#include <Arduino.h>
#include "ld19_reader.h"
#include "ld19_parser.h"
#include "ld19_udp.h"

//////////////////// USER CONFIG ////////////////////
const char* WIFI_SSID     = "Demon_net";
const char* WIFI_PASSWORD = "nopassword123";
const char* PC_IP         = "192.168.115.150";   // <-- set to your computer's IP
const uint16_t UDP_PORT   = 5005;             // <-- choose a port to listen on PC

// Mode selection - uncomment ONE of these
// #define MODE_RAW_HEX        // Mode 1: Raw hex dump
// #define MODE_PARSED_FRAMES  // Mode 2: Parsed frame display
#define MODE_UDP_FORWARD    // Mode 3: UDP forwarding

// Optional: Combination modes
// #define MODE_COMBO_PARSE_UDP // Mode 4: Parse AND forward via UDP
/////////////////////////////////////////////////////

// Global objects
LD19Reader* reader = nullptr;
LD19Parser* parser = nullptr;
LD19UDP* udp_sender = nullptr;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== LD19 LIDAR Multi-Mode Controller ===");
    
    // Initialize reader (always needed)
    reader = new LD19Reader();
    if (!reader->begin(230400)) {
        Serial.println("ERROR: Failed to initialize LIDAR reader!");
        return;
    }
    Serial.println("✓ LIDAR reader initialized");
    
    #ifdef MODE_RAW_HEX
        Serial.println("Mode: Raw hex dump");
        Serial.println("Starting raw hex output...\n");
    #endif
    
    #ifdef MODE_PARSED_FRAMES
        Serial.println("Mode: Parsed frame display");
        parser = new LD19Parser(reader->getSerial());
        Serial.println("✓ Frame parser initialized");
        Serial.println("Starting parsed frame output...\n");
    #endif
    
    #ifdef MODE_UDP_FORWARD
        Serial.println("Mode: UDP forwarding");
        udp_sender = new LD19UDP(WIFI_SSID, WIFI_PASSWORD, PC_IP, UDP_PORT);
        udp_sender->setSerial(reader->getSerial());
        
        if (!udp_sender->connectWiFi(WIFI_SSID, WIFI_PASSWORD)) {
            Serial.println("ERROR: WiFi connection failed!");
            return;
        }
        
        if (!udp_sender->setupUDP()) {
            Serial.println("ERROR: UDP setup failed!");
            return;
        }
        
        udp_sender->printStatus();
        Serial.println("Starting UDP forwarding...\n");
    #endif
    
    #ifdef MODE_COMBO_PARSE_UDP
        Serial.println("Mode: Parse AND UDP forwarding");
        
        // Initialize parser
        parser = new LD19Parser(reader->getSerial());
        Serial.println("✓ Frame parser initialized");
        
        // Initialize UDP
        udp_sender = new LD19UDP(WIFI_SSID, WIFI_PASSWORD, PC_IP, UDP_PORT);
        udp_sender->setSerial(reader->getSerial());
        
        if (!udp_sender->connectWiFi(WIFI_SSID, WIFI_PASSWORD)) {
            Serial.println("ERROR: WiFi connection failed!");
            return;
        }
        
        if (!udp_sender->setupUDP()) {
            Serial.println("ERROR: UDP setup failed!");
            return;
        }
        
        udp_sender->printStatus();
        Serial.println("Starting combined parse + UDP mode...\n");
    #endif
    
    Serial.println("Setup complete. Starting main loop...");
}

void loop() {
    #ifdef MODE_RAW_HEX
        // Mode 1: Simple raw hex output
        if (reader) {
            reader->printRawHex();
        }
    #endif
    
    #ifdef MODE_PARSED_FRAMES
        // Mode 2: Parse and display frames
        if (parser) {
            LD19Frame frame;
            if (parser->readFrame(frame)) {
                parser->printFrame(frame);
            }
        }
    #endif
    
    #ifdef MODE_UDP_FORWARD
        // Mode 3: Forward frames via UDP
        if (udp_sender) {
            udp_sender->forwardingLoop();
        }
    #endif
    
    #ifdef MODE_COMBO_PARSE_UDP
        // Mode 4: Parse frames and also forward via UDP
        if (parser && udp_sender) {
            // Try to get raw frame for UDP
            uint8_t raw_frame[47];
            if (parser->getRawFrame(raw_frame)) {
                // Send via UDP
                udp_sender->sendRawFrame(raw_frame, 47);
                
                // Also parse and display (but re-read from buffer)
                // Note: In a real application, you might want to parse from the raw_frame buffer
                // For demonstration, we'll just indicate that a frame was processed
                static int frame_count = 0;
                frame_count++;
                if (frame_count % 10 == 0) { // Print status every 10 frames
                    Serial.print("Processed ");
                    Serial.print(frame_count);
                    Serial.println(" frames (parse + UDP)");
                }
            }
        }
    #endif
    
    // Small delay to prevent overwhelming the system
    delay(1);
}

// Utility functions that can be called independently

void switchToRawMode() {
    Serial.println("Switching to raw hex mode...");
    // Implementation would reinitialize in raw mode
}

void switchToParseMode() {
    Serial.println("Switching to parse mode...");
    // Implementation would reinitialize in parse mode
}

void switchToUDPMode() {
    Serial.println("Switching to UDP mode...");
    // Implementation would reinitialize in UDP mode
}

void printSystemStatus() {
    Serial.println("\n=== System Status ===");
    Serial.print("Reader: ");
    Serial.println(reader ? "OK" : "NULL");
    Serial.print("Parser: ");
    Serial.println(parser ? "OK" : "NULL");
    Serial.print("UDP Sender: ");
    Serial.println(udp_sender ? "OK" : "NULL");
    
    if (udp_sender) {
        udp_sender->printStatus();
    }
    Serial.println("====================");
}

// Independent function examples
bool testLidarConnection() {
    if (!reader) return false;
    
    Serial.println("Testing LIDAR connection...");
    uint32_t start_time = millis();
    uint32_t byte_count = 0;
    
    while (millis() - start_time < 1000) { // Test for 1 second
        if (reader->dataAvailable()) {
            reader->readByte();
            byte_count++;
        }
        delay(1);
    }
    
    Serial.print("Received ");
    Serial.print(byte_count);
    Serial.println(" bytes in 1 second");
    
    return byte_count > 0;
}

bool testFrameParsing() {
    if (!parser) return false;
    
    Serial.println("Testing frame parsing...");
    uint32_t start_time = millis();
    int frame_count = 0;
    
    while (millis() - start_time < 5000 && frame_count < 5) { // Test for 5 seconds or 5 frames
        LD19Frame frame;
        if (parser->readFrame(frame)) {
            frame_count++;
            Serial.print("Frame ");
            Serial.print(frame_count);
            Serial.print(": Speed=");
            Serial.print(frame.speed_dps);
            Serial.print(" dps, Points=12");
            Serial.println();
        }
        delay(10);
    }
    
    Serial.print("Successfully parsed ");
    Serial.print(frame_count);
    Serial.println(" frames");
    
    return frame_count > 0;
}

bool testUDPTransmission() {
    if (!udp_sender) return false;
    
    Serial.println("Testing UDP transmission...");
    
    if (!udp_sender->isReady()) {
        Serial.println("UDP sender not ready!");
        return false;
    }
    
    uint32_t start_time = millis();
    int sent_count = 0;
    
    while (millis() - start_time < 5000 && sent_count < 10) { // Test for 5 seconds or 10 frames
        if (udp_sender->readAndForwardFrame()) {
            sent_count++;
        }
        delay(10);
    }
    
    Serial.print("Successfully sent ");
    Serial.print(sent_count);
    Serial.println(" frames via UDP");
    
    return sent_count > 0;
}