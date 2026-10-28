#include "stewart_safety.h"

SafetyMonitor::SafetyMonitor()
    : lastCommMs(0), alarmCode(0), eStopActive(false) {}

void SafetyMonitor::setLimits(const SafetyLimits& lim) {
    limits = lim;
}

void SafetyMonitor::updateLegLengths(const float lengths[6]) {
    bool violation = false;
    for (int i = 0; i < 6; ++i) {
        if (lengths[i] < limits.legMin[i] || lengths[i] > limits.legMax[i]) {
            violation = true;
            break;
        }
    }
    if (violation) alarmCode |= 0x01;
    else           alarmCode &= ~0x01;
}

void SafetyMonitor::updateCondition(float cond) {
    if (cond < limits.minCondition) alarmCode |= 0x02;
    else                              alarmCode &= ~0x02;
}

void SafetyMonitor::feedComm() {
    lastCommMs = millis();
    alarmCode &= ~0x04;
}

bool SafetyMonitor::checkEStop(uint8_t pin) {
    eStopActive = (digitalRead(pin) == LOW); // active-low with pull-up
    if (eStopActive) alarmCode |= 0x08;
    else             alarmCode &= ~0x08;
    return eStopActive;
}

uint32_t SafetyMonitor::elapsedSinceComm() const {
    return millis() - lastCommMs;
}

bool SafetyMonitor::isSafe() const {
    if (alarmCode) return false;
    if (elapsedSinceComm() > limits.commTimeoutMs) return false;
    return true;
}
