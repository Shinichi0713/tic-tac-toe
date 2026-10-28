#pragma once
#include "stewart_geometry.h"
#include <Arduino.h>

class CommProtocol {
public:
    void begin(HardwareSerial& serial, long baud);
    bool parseTarget(Pose6& out);
    void sendStatus(const float legLengths[6], uint8_t alarm, const Pose6* pose);
private:
    HardwareSerial* serial;
};
