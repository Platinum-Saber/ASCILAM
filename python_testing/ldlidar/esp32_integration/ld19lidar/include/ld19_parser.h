#ifndef LD19_PARSER_H
#define LD19_PARSER_H

#include <Arduino.h>
#include <HardwareSerial.h>

// LD19 frame parsing and formatting functionality
struct LD19Point {
    uint16_t distance_mm;
    uint8_t intensity;
};

struct LD19Frame {
    uint16_t speed_dps;
    uint16_t start_angle;  // hundredths of degree
    uint16_t end_angle;    // hundredths of degree
    uint16_t timestamp_ms;
    uint8_t crc8;
    LD19Point points[12];
    bool valid;
};

class LD19Parser {
private:
    static const uint8_t LD19_HEADER = 0x54;
    static const uint8_t LD19_VERLEN = 0x2C;
    static const size_t LD19_FRAME_SIZE = 47;
    
    HardwareSerial* lidar_serial;
    
    // Helper function to read exact number of bytes with timeout
    bool readExact(uint8_t* buf, size_t n, uint32_t timeout_ms = 25);
    
    // Helper function to extract 16-bit values from frame
    uint16_t extractU16(const uint8_t* frame, int index);

public:
    LD19Parser();
    LD19Parser(HardwareSerial* serial);
    
    // Set the serial interface
    void setSerial(HardwareSerial* serial);
    
    // Find and sync to frame header
    bool findHeader();
    
    // Read and parse a complete frame
    bool readFrame(LD19Frame& frame);
    
    // Print formatted frame data
    void printFrame(const LD19Frame& frame);
    
    // Get raw frame data (47 bytes)
    bool getRawFrame(uint8_t* buffer);
    
    // Validate frame integrity
    bool validateFrame(const uint8_t* frame);
};

#endif // LD19_PARSER_H
