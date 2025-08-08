#include "ld19_udp.h"

LD19UDP::LD19UDP() : wifi_connected(false), udp_initialized(false), lidar_serial(nullptr) {
}

LD19UDP::LD19UDP(const char* ssid, const char* password, const char* ip, uint16_t port) 
    : wifi_ssid(ssid), wifi_password(password), target_ip(ip), target_port(port),
      wifi_connected(false), udp_initialized(false), lidar_serial(nullptr) {
}

LD19UDP::~LD19UDP() {
    disconnectWiFi();
}

bool LD19UDP::connectWiFi(const char* ssid, const char* password) {
    wifi_ssid = ssid;
    wifi_password = password;
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting WiFi");
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    wifi_connected = (WiFi.status() == WL_CONNECTED);
    
    if (wifi_connected) {
        Serial.print("\nWiFi OK. IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nWiFi connection failed!");
    }
    
    return wifi_connected;
}

bool LD19UDP::isWiFiConnected() {
    wifi_connected = (WiFi.status() == WL_CONNECTED);
    return wifi_connected;
}

void LD19UDP::disconnectWiFi() {
    WiFi.disconnect();
    wifi_connected = false;
}

IPAddress LD19UDP::getLocalIP() {
    return WiFi.localIP();
}

bool LD19UDP::setupUDP() {
    if (!wifi_connected) return false;
    
    udp_initialized = udp.begin(0); // ephemeral local port
    return udp_initialized;
}

void LD19UDP::setTarget(const char* ip, uint16_t port) {
    target_ip = ip;
    target_port = port;
}

String LD19UDP::getTargetIP() {
    return target_ip;
}

uint16_t LD19UDP::getTargetPort() {
    return target_port;
}

void LD19UDP::setSerial(HardwareSerial* serial) {
    lidar_serial = serial;
}

bool LD19UDP::findHeader() {
    if (!lidar_serial) return false;
    
    // Seek for 0x54 (header). Non-blocking but quick.
    while (lidar_serial->available()) {
        if (lidar_serial->read() == LD19_HEADER) return true;
    }
    return false;
}

bool LD19UDP::readExact(uint8_t* buf, size_t len) {
    if (!lidar_serial) return false;
    
    uint32_t t0 = millis();
    size_t got = 0;
    while (got < len) {
        int c = lidar_serial->read();
        if (c >= 0) {
            buf[got++] = (uint8_t)c;
        } else {
            if (millis() - t0 > 20) { // short timeout to avoid stalling
                return false;
            }
            delayMicroseconds(200);
        }
    }
    return true;
}

bool LD19UDP::sendRawFrame(const uint8_t* frame, size_t frame_size) {
    if (!isReady() || !frame) return false;
    
    udp.beginPacket(target_ip.c_str(), target_port);
    size_t sent = udp.write(frame, frame_size);
    bool success = udp.endPacket();
    
    return (success && sent == frame_size);
}

bool LD19UDP::readAndForwardFrame() {
    if (!isReady() || !lidar_serial) return false;
    
    // Try to align on a full frame starting at header
    if (!findHeader()) {
        // no data right now
        return false;
    }
    
    // We already consumed 1 header byte; store it and pull the rest
    uint8_t frame[LD19_FRAME_SIZE];
    frame[0] = LD19_HEADER;
    
    if (!readExact(frame + 1, LD19_FRAME_SIZE - 1)) {
        // failed to fill a full frame; desync
        return false;
    }
    
    // Optional: quick sanity check (verlen should be 0x2C for 12 points)
    if (frame[1] != 0x2C) {
        // Not a standard measurement frame; drop & resync
        return false;
    }
    
    // Send raw frame via UDP
    return sendRawFrame(frame, LD19_FRAME_SIZE);
}

void LD19UDP::forwardingLoop() {
    if (!readAndForwardFrame()) {
        // no data or failed to send, small delay to prevent busy waiting
        delay(1);
    }
}

void LD19UDP::printStatus() {
    Serial.println("\n=== LD19 UDP Status ===");
    Serial.print("WiFi Connected: ");
    Serial.println(isWiFiConnected() ? "Yes" : "No");
    if (wifi_connected) {
        Serial.print("Local IP: ");
        Serial.println(WiFi.localIP());
    }
    Serial.print("UDP Initialized: ");
    Serial.println(udp_initialized ? "Yes" : "No");
    Serial.print("Target: ");
    Serial.print(target_ip);
    Serial.print(":");
    Serial.println(target_port);
    Serial.print("LIDAR Serial: ");
    Serial.println(lidar_serial ? "Set" : "Not Set");
    Serial.print("Ready: ");
    Serial.println(isReady() ? "Yes" : "No");
    Serial.println("=====================");
}

bool LD19UDP::isReady() {
    return isWiFiConnected() && udp_initialized && lidar_serial;
}
