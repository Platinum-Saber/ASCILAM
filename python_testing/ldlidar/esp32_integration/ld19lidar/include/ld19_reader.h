#ifndef LD19_READER_H
#define LD19_READER_H

#include <Arduino.h>
#include <HardwareSerial.h>

// LD19 raw serial reader functionality
class LD19Reader {
private:
    HardwareSerial* lidar_serial;
    static const int PIN_RX = 16;
    static const int PIN_TX = 17;
    uint32_t byte_count;

public:
    LD19Reader();
    ~LD19Reader();
    
    // Initialize the LIDAR serial connection
    bool begin(uint32_t baud_rate = 230400);
    
    // Read and print raw hex data
    void printRawHex();
    
    // Check if data is available
    bool dataAvailable();
    
    // Read a single byte
    uint8_t readByte();
    
    // Get the HardwareSerial instance for advanced usage
    HardwareSerial* getSerial();
};

#endif // LD19_READER_H
