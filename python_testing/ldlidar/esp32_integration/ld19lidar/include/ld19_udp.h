#ifndef LD19_UDP_H
#define LD19_UDP_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>

// LD19 UDP transmission functionality
class LD19UDP {
private:
    WiFiUDP udp;
    String wifi_ssid;
    String wifi_password;
    String target_ip;
    uint16_t target_port;
    bool wifi_connected;
    bool udp_initialized;
    
    HardwareSerial* lidar_serial;
    static const uint8_t LD19_HEADER = 0x54;
    static const size_t LD19_FRAME_SIZE = 47;
    
    // Helper functions
    bool findHeader();
    bool readExact(uint8_t* buf, size_t len);

public:
    LD19UDP();
    LD19UDP(const char* ssid, const char* password, const char* ip, uint16_t port);
    ~LD19UDP();
    
    // WiFi connection management
    bool connectWiFi(const char* ssid, const char* password);
    bool isWiFiConnected();
    void disconnectWiFi();
    IPAddress getLocalIP();
    
    // UDP configuration
    bool setupUDP();
    void setTarget(const char* ip, uint16_t port);
    String getTargetIP();
    uint16_t getTargetPort();
    
    // LIDAR serial interface
    void setSerial(HardwareSerial* serial);
    
    // Data transmission
    bool sendRawFrame(const uint8_t* frame, size_t frame_size);
    bool readAndForwardFrame();
    
    // Main loop function for continuous forwarding
    void forwardingLoop();
    
    // Status functions
    void printStatus();
    bool isReady(); // WiFi connected and UDP initialized
};

#endif // LD19_UDP_H
