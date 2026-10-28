#include "stewart_control.h"

AxisController::AxisController()
    : targetPos(0), currentPos(0), lastError(0), integrator(0), stepAccum(0) {}

void AxisController::init(const AxisConfig& cfg_) {
    cfg = cfg_;
    pinMode(cfg.pinPUL, OUTPUT);
    pinMode(cfg.pinDIR, OUTPUT);
    digitalWriteFast(cfg.pinPUL, LOW);
    digitalWriteFast(cfg.pinDIR, LOW);
    reset();
}

void AxisController::reset() {
    lastError = 0;
    integrator = 0;
    stepAccum = 0;
}

void AxisController::setTarget(float posMM) {
    targetPos = posMM;
}

void AxisController::setCurrentPosition(float mm) {
    currentPos = mm;
    stepAccum = 0;
}

void AxisController::update(float dt) {
    float error = targetPos - currentPos;
    integrator += error * dt;
    if (integrator > cfg.iMax) integrator = cfg.iMax;
    if (integrator < cfg.iMin) integrator = cfg.iMin;

    float derivative = (error - lastError) / dt;
    float velCmd = cfg.Kp * error + cfg.Ki * integrator + cfg.Kd * derivative;
    lastError = error;

    if (velCmd >  cfg.maxVel) velCmd =  cfg.maxVel;
    if (velCmd < -cfg.maxVel) velCmd = -cfg.maxVel;

    float stepsPerSec = velCmd * cfg.stepsPerMM;
    stepAccum += stepsPerSec * dt;

    int steps = (int)stepAccum;
    if (steps != 0) {
        stepAccum -= (float)steps;
        digitalWriteFast(cfg.pinDIR, steps > 0 ? HIGH : LOW);
        int n = abs(steps);
        for (int s = 0; s < n; ++s) {
            digitalWriteFast(cfg.pinPUL, HIGH);
            delayNanoseconds(800);
            digitalWriteFast(cfg.pinPUL, LOW);
            delayNanoseconds(800);
        }
        currentPos += (float)steps / cfg.stepsPerMM;
    }
}

void StepperManager::init() {
    // Example: NEMA23 1.8deg, 16 microstepping, ballscrew lead 5mm
    // 200 steps/rev * 16 usteps / 5mm = 640 steps/mm
    AxisConfig base = {0, 0, 640.0f, 80.0f, 20.0f, 0.2f, -200.0f, 200.0f, 30.0f};
    uint8_t pulPins[6] = {2, 4, 6, 8, 10, 24};
    uint8_t dirPins[6] = {3, 5, 7, 9, 11, 25};
    for (int i = 0; i < 6; ++i) {
        base.pinPUL = pulPins[i];
        base.pinDIR = dirPins[i];
        axes[i].init(base);
        currentLengths[i] = 0;
    }
}

void StepperManager::setTargetLengths(const float lengths[6]) {
    for (int i = 0; i < 6; ++i) {
        axes[i].setTarget(lengths[i]);
    }
}

void StepperManager::setCurrentLengths(const float lengths[6]) {
    for (int i = 0; i < 6; ++i) {
        currentLengths[i] = lengths[i];
        axes[i].setCurrentPosition(lengths[i]);
    }
}

void StepperManager::update1kHz(float dt) {
    for (int i = 0; i < 6; ++i) {
        axes[i].update(dt);
        currentLengths[i] = axes[i].getPositionMM();
    }
}
