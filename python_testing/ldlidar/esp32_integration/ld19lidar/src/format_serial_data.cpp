#include "ld19_parser.h"

LD19Parser::LD19Parser() : lidar_serial(nullptr) {
}

LD19Parser::LD19Parser(HardwareSerial* serial) : lidar_serial(serial) {
}

void LD19Parser::setSerial(HardwareSerial* serial) {
    lidar_serial = serial;
}

bool LD19Parser::readExact(uint8_t* buf, size_t n, uint32_t timeout_ms) {
    if (!lidar_serial) return false;
    
    uint32_t t0 = millis();
    size_t got = 0;
    while (got < n) {
        int c = lidar_serial->read();
        if (c >= 0) {
            buf[got++] = (uint8_t)c;
        } else {
            if (millis() - t0 > timeout_ms) return false;
            delayMicroseconds(200);
        }
    }
    return true;
}

uint16_t LD19Parser::extractU16(const uint8_t* frame, int index) {
    return (uint16_t)frame[index] | ((uint16_t)frame[index + 1] << 8);
}

bool LD19Parser::findHeader() {
    if (!lidar_serial) return false;
    
    // Seek for 0x54 (header)
    int c = lidar_serial->read();
    if (c < 0) return false;
    return ((uint8_t)c == LD19_HEADER);
}

bool LD19Parser::readFrame(LD19Frame& frame) {
    if (!lidar_serial) return false;
    
    // Try to find header
    if (!findHeader()) return false;
    
    uint8_t raw_frame[LD19_FRAME_SIZE];
    raw_frame[0] = LD19_HEADER;
    
    if (!readExact(raw_frame + 1, LD19_FRAME_SIZE - 1)) {
        frame.valid = false;
        return false;
    }
    
    // Quick sanity: VerLen must be 0x2C for 12 points
    if (raw_frame[1] != LD19_VERLEN) {
        frame.valid = false;
        return false;
    }
    
    // Parse frame data
    frame.speed_dps = extractU16(raw_frame, 2);
    frame.start_angle = extractU16(raw_frame, 4);
    frame.end_angle = extractU16(raw_frame, 42);
    frame.timestamp_ms = extractU16(raw_frame, 44);
    frame.crc8 = raw_frame[46];
    
    // Parse the 12 points
    int offset = 6;
    for (int i = 0; i < 12; ++i) {
        frame.points[i].distance_mm = extractU16(raw_frame, offset);
        frame.points[i].intensity = raw_frame[offset + 2];
        offset += 3;
    }
    
    frame.valid = true;
    return true;
}

void LD19Parser::printFrame(const LD19Frame& frame) {
    if (!frame.valid) {
        Serial.println("Invalid frame!");
        return;
    }
    
    Serial.print("\nPacket: spd=");
    Serial.print(frame.speed_dps);
    Serial.print(" dps  start=");
    Serial.print(frame.start_angle / 100.0, 2);
    Serial.print("°  end=");
    Serial.print(frame.end_angle / 100.0, 2);
    Serial.print("°  ts=");
    Serial.print(frame.timestamp_ms);
    Serial.print(" ms  (crc=0x");
    if (frame.crc8 < 16) Serial.print('0');
    Serial.print(frame.crc8, HEX);
    Serial.println(")");
    
    // Print the 12 points
    for (int i = 0; i < 12; ++i) {
        Serial.print("  pt");
        if (i < 10) Serial.print('0');
        Serial.print(i);
        Serial.print(": d=");
        Serial.print(frame.points[i].distance_mm);
        Serial.print("mm  I=");
        Serial.println(frame.points[i].intensity);
    }
}

bool LD19Parser::getRawFrame(uint8_t* buffer) {
    if (!lidar_serial || !buffer) return false;
    
    // Try to find header
    if (!findHeader()) return false;
    
    buffer[0] = LD19_HEADER;
    return readExact(buffer + 1, LD19_FRAME_SIZE - 1);
}

bool LD19Parser::validateFrame(const uint8_t* frame) {
    if (!frame) return false;
    
    // Check header and verlen
    return (frame[0] == LD19_HEADER && frame[1] == LD19_VERLEN);
}
