#pragma once
#include <Arduino.h>

struct SafetyLimits {
    float legMin[6];
    float legMax[6];
    float minCondition; // 0..1 (1=healthy)
    uint32_t commTimeoutMs;
};

class SafetyMonitor {
public:
    SafetyMonitor();
    void setLimits(const SafetyLimits& lim);
    void updateLegLengths(const float lengths[6]);
    void updateCondition(float cond);
    void feedComm();
    bool checkEStop(uint8_t pin);
    bool isSafe() const;
    uint8_t getAlarm() const { return alarmCode; }
    uint32_t elapsedSinceComm() const;
private:
    SafetyLimits limits;
    uint32_t lastCommMs;
    uint8_t alarmCode; // bit0=leg limit, bit1=singularity, bit2=comm, bit3=estop
    bool eStopActive;
};
