#include "Arduino.h"
#include "motor.h"

static void delay_1us();
static float clip(float val, float maxVal);

Motor::Motor(int stepPin, int dirPin, int updatesPerSec, bool forward) {
    _stepPin = stepPin;
    _dirPin = dirPin;
    _forward = forward;
    _maxStepperAcceleration = 1600/updatesPerSec;
    _speed = 0;
    _nextStep = 0;
    _usecBetweenSteps = 0;
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
}

int16_t Motor::CurrentSpeed() {
    return (int16_t)((float)_speed/stepsInRotation*360);
}

#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

void Motor::Run() {
    // This will wrap in 71 minutes and I need to fix it.
    unsigned long now = micros();
    if (now > _nextStep) {
        if (_usecBetweenSteps > 0) {
            digitalWrite(_stepPin, HIGH);
            delay_1us();
            digitalWrite(_stepPin, LOW);
            _nextStep = now + _usecBetweenSteps;
        }
    }
}

void Motor::SetSpeed(int16_t newSpeed) {
    // Clip to our maximum speed.
    newSpeed = clip(newSpeed, maxMotorSpeed);
    int16_t desiredSpeed = (int16_t)(((float)newSpeed/360.0)*stepsInRotation);

    // Limit the maximum acceleration of the motors
    if ((_speed - desiredSpeed) > _maxStepperAcceleration) {
        _speed -= _maxStepperAcceleration;
    } else if ((_speed - desiredSpeed) < -_maxStepperAcceleration) {
        _speed += _maxStepperAcceleration;
    } else {
        _speed = desiredSpeed;
    }
    int16_t temp = _speed;
    if (temp > 0) {
        digitalWrite(_dirPin, _forward ? HIGH : LOW);
    } else if (temp < 0){
        digitalWrite(_dirPin, _forward ? LOW : HIGH);
        temp = -temp;
    }
    if (temp == 0) {
        _usecBetweenSteps = 0;
    } else {
        _usecBetweenSteps = 1000000 / (temp * use16steps);
    }
}

static float clip(float val, float maxVal) {
    if (val < -maxVal) {
        val = -maxVal;
    } else if (val > maxVal) {
        val = maxVal;
    }
    return val;
}

// 16 single cycle instructions at 16Mhz equals 1us.
static void delay_1us() {
 __asm__ __volatile__ (
   "nop" "\n\t"
   "nop" "\n\t"
   "nop" "\n\t"
   "nop" "\n\t"
   "nop" "\n\t"
   "nop" "\n\t"
   "nop" "\n\t"
   "nop" "\n\t"
   "nop" "\n\t"
   "nop" "\n\t"
   "nop" "\n\t"
   "nop" "\n\t"
   "nop" "\n\t"
   "nop" "\n\t"
   "nop" "\n\t"
   "nop");
}
