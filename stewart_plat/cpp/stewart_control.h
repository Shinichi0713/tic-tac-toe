#pragma once
#include "stewart_geometry.h"

struct AxisConfig {
    uint8_t pinPUL;
    uint8_t pinDIR;
    float stepsPerMM;   // steps per mm (include microstepping)
    float Kp, Ki, Kd;
    float iMin, iMax;   // integrator clamp
    float maxVel;       // mm/s
};

class AxisController {
public:
    AxisController();
    void init(const AxisConfig& cfg);
    void setTarget(float posMM);
    void setCurrentPosition(float mm);
    void reset();
    void update(float dt); // call at 1kHz
    float getPositionMM() const { return currentPos; }
    float getTargetMM()   const { return targetPos; }
private:
    AxisConfig cfg;
    float targetPos;
    float currentPos;
    float lastError;
    float integrator;
    float stepAccum;
};

class StepperManager {
public:
    void init();
    void setTargetLengths(const float lengths[6]);
    void setCurrentLengths(const float lengths[6]);
    void update1kHz(float dt);
    const float* getCurrentLengths() const { return currentLengths; }
private:
    AxisController axes[6];
    float currentLengths[6];
};
