#ifndef RADIOLINK_M10UBX_GNSS_H
#define RADIOLINK_M10UBX_GNSS_H

#include <Arduino.h>

struct GNSSData {
    double lat, lon;         // Changed to double for high precision
    float speedKmh, speedMph;
    float heading, headAcc;
    float hAcc, vAcc, hdop;
    bool  sbasUsed, headingValid;
    int   sats, fixType;
    uint32_t lastUpdate;
    uint16_t year;
    uint8_t  month, day, hour, minute, second;
};

class RadiolinkM10UBX {
public:
    RadiolinkM10UBX(HardwareSerial& serial);
    void begin(unsigned long baud = 38400, int rxPin = 16, int txPin = 17);
    bool update(); 
    GNSSData data;

private:
    HardwareSerial* _serial;
    uint8_t _state, _msgClass, _msgID, _idx;
    uint16_t _payloadLen;
    uint8_t _payload[128];
    uint8_t _ckA, _ckB; // Internal checksum storage
    
    void sendUBX(const uint8_t* payload, uint16_t len);
    void parse(uint8_t c);
    bool checkChecksum(uint8_t classID, uint8_t msgID, uint16_t length, uint8_t* payload, uint8_t ckA, uint8_t ckB);
    bool _newPacketAvailable;
};

#endif