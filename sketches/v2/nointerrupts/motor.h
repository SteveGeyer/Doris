#ifndef motor_h
#define motor_h

const int stepsInRotation = 200; // Number of steps per rotation.
const int maxMotorSpeed = 8*360; // Maximum motor speed in degrees/sec.
const int use16steps = 16;       // Multiplier for 16 microsteps.

class Motor {
  public:
    Motor(int stepPin, int dirPin, int updatesPerSec, bool forward);  // Initialize motor.
    int16_t CurrentSpeed();          // Current speed in degrees/sec.
    void Run();                      // Run motor. Does not block.
    void SetSpeed(int speed);        // Speed in degrees/sec.

  private:
    uint8_t _stepPin;                // Pin to step motor.
    uint8_t _dirPin;                 // Pin that sets direction.
    bool _forward;                   // Run in forward direction.
    int16_t _speed;                  // Current speed in ticks/sec.
    int16_t _usecBetweenSteps;       // Time between steps.
    uint32_t _nextStep;              // Next time step.
    int16_t _maxStepperAcceleration; // How fast can we accelerate.
};

#endif // motor_h
