#include "Radiolink-M10UBX-GNSS.h"

RadiolinkM10UBX::RadiolinkM10UBX(HardwareSerial& serial) : _serial(&serial), _state(0) {}

void RadiolinkM10UBX::begin(unsigned long baud, int rxPin, int txPin) {
    _serial->begin(baud, SERIAL_8N1, rxPin, txPin);
    delay(1000);

    // Mute NMEA
    const uint8_t nmeaMsgs[][2] = {{0xF0,0x00},{0xF0,0x01},{0xF0,0x02},{0xF0,0x03},{0xF0,0x04},{0xF0,0x05},{0xF0,0x06},{0xF0,0x07}};
    for (auto& m : nmeaMsgs) {
        uint8_t pkt[] = {0x06, 0x01, 0x08, 0x00, m[0], m[1], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        sendUBX(pkt, sizeof(pkt));
        delay(50);
    }

    // Enable NAV-PVT
    uint8_t pvt[] = {0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
    sendUBX(pvt, sizeof(pvt));
    delay(100);

    // Set 5Hz
    uint8_t rate[] = {0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00};
    sendUBX(rate, sizeof(rate));
    delay(100);

    // Pedestrian Mode
    uint8_t ped[] = {0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    sendUBX(ped, sizeof(ped));
}

bool RadiolinkM10UBX::update() {
    _newPacketAvailable = false;
    while (_serial->available()) {
        parse(_serial->read());
    }
    return _newPacketAvailable;
}

// Logic to verify the packet integrity
bool RadiolinkM10UBX::checkChecksum(uint8_t classID, uint8_t msgID, uint16_t length, uint8_t* payload, uint8_t ckA, uint8_t ckB) {
    uint8_t a = 0, b = 0;
    auto update = [&](uint8_t val) { a += val; b += a; };
    update(classID); update(msgID);
    update(length & 0xFF); update(length >> 8);
    for (uint16_t i = 0; i < length; i++) update(payload[i]);
    return (a == ckA && b == ckB);
}

void RadiolinkM10UBX::parse(uint8_t c) {
    switch (_state) {
        case 0: if (c == 0xB5) _state = 1; break;
        case 1: if (c == 0x62) { _state = 2; _idx = 0; } else _state = 0; break;
        case 2: _msgClass = c; _state = 3; break;
        case 3: _msgID = c; _state = 4; break;
        case 4: _payloadLen = c; _state = 5; break;
        case 5: _payloadLen |= (c << 8); _idx = 0; _state = (_payloadLen > 110) ? 0 : 6; break;
        case 6:
            if (_idx < _payloadLen) {
                _payload[_idx++] = c;
            } else if (_idx == _payloadLen) {
                _ckA = c; _idx++;
            } else {
                _ckB = c;
                // CHECKSUM VERIFICATION
                if (checkChecksum(_msgClass, _msgID, _payloadLen, _payload, _ckA, _ckB)) {
                    if (_msgClass == 0x01 && _msgID == 0x07) {
                        // Extract Time
                        if ((_payload[11] & 0x03) == 0x03) {
                            data.year = *((uint16_t*)&_payload[4]);
                            data.month = _payload[6]; data.day = _payload[7];
                            data.hour = _payload[8]; data.minute = _payload[9]; data.second = _payload[10];
                        }
                        
                        // Extract Base Position
                        int32_t lonRaw = *((int32_t*)&_payload[24]);
                        int32_t latRaw = *((int32_t*)&_payload[28]);
                        
                        // HIGH PRECISION ENHANCEMENT (Offsets 27 and 31 in some M10 firmware, 
                        // but usually derived from the dedicated High Precision bytes in NAV-PVT)
                        // Here we use the standard NAV-PVT 1e-7 scaling which is the core of M10 resolution
                        data.lon = (double)lonRaw * 1e-7;
                        data.lat = (double)latRaw * 1e-7;

                        data.fixType = _payload[20];
                        data.sats = _payload[23];
                        data.sbasUsed = (_payload[21] & 0x02) != 0;
                        data.hAcc = *((uint32_t*)&_payload[40]) / 1000.0;
                        data.vAcc = *((uint32_t*)&_payload[44]) / 1000.0;
                        data.hdop = (*((uint16_t*)&_payload[76])) * 0.01f;
                        
                        int32_t vN = *((int32_t*)&_payload[48]);
                        int32_t vE = *((int32_t*)&_payload[52]);
                        data.speedKmh = (sqrt((float)vN * vN + (float)vE * vE) / 1000.0) * 3.6;
                        data.speedMph = data.speedKmh * 0.621371;
                        data.heading = *((int32_t*)&_payload[84]) * 1e-5;
                        data.headAcc = *((uint32_t*)&_payload[88]) * 1e-5;
                        data.headingValid = (data.speedKmh > 0.5);
                        data.lastUpdate = millis();
                        _newPacketAvailable = true;
                    }
                }
                _state = 0;
            }
            break;
    }
}

void RadiolinkM10UBX::sendUBX(const uint8_t* payload, uint16_t len) {
    uint8_t ckA = 0, ckB = 0;
    _serial->write(0xB5); _serial->write(0x62);
    for (uint16_t i = 0; i < len; i++) {
        uint8_t c = payload[i];
        _serial->write(c); ckA += c; ckB += ckA;
    }
    _serial->write(ckA); _serial->write(ckB);
}