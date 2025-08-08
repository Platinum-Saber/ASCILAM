#include "ld19_reader.h"

LD19Reader::LD19Reader() : byte_count(0) {
    lidar_serial = new HardwareSerial(2);
}

LD19Reader::~LD19Reader() {
    if (lidar_serial) {
        delete lidar_serial;
    }
}

bool LD19Reader::begin(uint32_t baud_rate) {
    if (!lidar_serial) return false;
    
    // Initialize LIDAR serial connection
    lidar_serial->begin(baud_rate, SERIAL_8N1, PIN_RX, PIN_TX, false);
    delay(100); // Allow time for initialization
    
    return true;
}

void LD19Reader::printRawHex() {
    // Print bytes as hex as they arrive
    while (lidar_serial && lidar_serial->available()) {
        uint8_t b = lidar_serial->read();
        if ((byte_count++ % 32) == 0) Serial.println();
        if (b < 0x10) Serial.print('0');
        Serial.print(b, HEX);
        Serial.print(' ');
    }
}

bool LD19Reader::dataAvailable() {
    return lidar_serial && lidar_serial->available();
}

uint8_t LD19Reader::readByte() {
    if (lidar_serial && lidar_serial->available()) {
        return lidar_serial->read();
    }
    return 0;
}

HardwareSerial* LD19Reader::getSerial() {
    return lidar_serial;
}
