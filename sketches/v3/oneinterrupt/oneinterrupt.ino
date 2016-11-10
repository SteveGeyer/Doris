#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// Pins
const int rcLeftRight = 2;
const int rcForwardBackward = 3;
const int buzzer = 4;
const int motorMS1 = 5;
const int motorMS2 = 6;
const int motorMS3 = 7;
const int frontDir = 8;
const int frontStep = 9;
const int leftDir = 10;
const int leftStep = 11;
const int rightDir = 12;
const int rightStep = 13;
const int motorEnable = A0;

// Physical constants.
const float rad2deg = 180/PI;
const float deg2rad = 1/rad2deg;

// Parameters gotten from MPU6050_calibration
const long xAccelOff = -1090;
const long yAccelOff = 1575;
const long zAccelOff = 1178;
const long xGyroOff = 50;
const long yGyroOff = -57;
const long zGyroOff = 11;

const float   maxAngle = 10;       // In degrees.
const float   stopMotorAngle = 15; // In degrees.
const int     updatesPerSec = 200;
const uint8_t gyroRange     = MPU6050_GYRO_FS_2000;
const uint8_t accelRange    = MPU6050_ACCEL_FS_2;
const float   switchToAccel = 0.5; // Secs before trusting accel.
const float   Kf            = switchToAccel/(switchToAccel+1.0/updatesPerSec);

const int      maxRotations = 4;                 // Maximum rotations/sec.
const int      stepsInRotation = 200;            // Number of steps per rotation.
const int      use16steps = 16;                  // Multiplier for 16 microsteps.
const int      maxMotorSpeed = maxRotations*360; // Maximum motor speed in degrees/sec.
const uint16_t slowestSpeed = 65535;             // Slowest speed for timer.
const long     twoMhz = 2000000;                 // Stepper 2Mhz constant.

const int      maxSteps = maxRotations * stepsInRotation * use16steps;

// Maximum stepper acceleration which is based on the update frequency.
const int      maxStepperAcceleration = 1600/updatesPerSec;

static int16_t m1Speed;         // Motor speed,
volatile int8_t  m1Running;     //  true if motors running,
volatile int16_t m1Counter;     //   interrupt counter,
volatile int16_t m1ResetValue;  //    and interrupt reset value.

static int16_t m2Speed;         // Motor speed,
volatile int8_t  m2Running;     //  true if motors running,
volatile int16_t m2Counter;     //   interrupt counter,
volatile int16_t m2ResetValue;  //    and interrupt reset value.

static MPU6050 mpu;             // Gyroscopes and accelerometers.
static float r2rs;              // Rotation int16 to radians/sec.
static float a2g;               // Acceleration int to G.
static float cfAngleRadians;    // Raw complementary filter angle in radians.

static float motorSpeed;        // Speed to command motors.

static float angleITerm;        // Angle PID iTerm.
static float angleLastInput;    // Angle PID last input (for dTerm).

static unsigned long nextSampleTime;

static float Kap;               // Speed to angle proportional.
static float Kai;               // Speed to angle integrative.
static float Kad;               // Speed to angle derivative.
static float Kbp;               // Balance proportional.
static float Kbd;               // Balance derivative.
static float balanceOffset;     // This is the balace angle for robot.

static unsigned long now;       // Last call to micros();

const unsigned long rcTimeOut = 51000;
const float rcLowPassConst = 0.985;
static int lastForwardBackwardValue;
static int lastLeftRightValue;
static unsigned long lastForwardBackwardTime;
static unsigned long lastLeftRightTime;
static int lowPassForwardBackward;
static int lowPassLeftRight;

static void delay_1us();
static bool nextSample();
static float clip(float val, float maxVal);
static void setMotorSpeedM1(int16_t tspeed);
static void setMotorSpeedM2(int16_t tspeed);

void setup() {
    Serial.begin(115200);

    Wire.begin();
    TWBR = 24;                  // 400kHz I2C clock (200kHz if CPU is 8MHz).

    // I seem to have to initialize the MPU twice to get stable results on
    // boot. I am not sure why.
    initializeMPU();
    initializeMPU();

    const float lsbSensitity = 32768.0/250.0;
    r2rs = ((1 << gyroRange) / lsbSensitity) * deg2rad;
    a2g = (1 << accelRange) / 16384.0;

    mpu.setXGyroOffset(xGyroOff);
    mpu.setYGyroOffset(yGyroOff);
    mpu.setZGyroOffset(zGyroOff);
    mpu.setXAccelOffset(xAccelOff);
    mpu.setYAccelOffset(yAccelOff);
    mpu.setZAccelOffset(zAccelOff);

    // Setup stepper motor values and pins.
    pinMode(motorEnable, OUTPUT); // Enable pin set to output.
    digitalWrite(motorEnable, LOW);
    pinMode(motorMS1, OUTPUT);
    digitalWrite(motorMS1, HIGH);
    pinMode(motorMS2, OUTPUT);
    digitalWrite(motorMS2, HIGH);
    pinMode(motorMS3, OUTPUT);
    digitalWrite(motorMS3, HIGH);

    // Setup stepper motor values and pins.
    m1Speed = 0;
    m1Running = 0;
    m1Counter = 0;
    m1ResetValue = 0;
    m2Speed = 0;
    m2Running = 0;
    m2Counter = 0;
    m2ResetValue = 0;
    pinMode(leftDir, OUTPUT);   // Motor 1 step porte,6
    pinMode(leftStep, OUTPUT);  // Motor 1 direction portb,4
    pinMode(rightDir, OUTPUT);  // Motor 2 step portd,6
    pinMode(rightStep, OUTPUT); // Motor 2 direction portc,6
    setMotorSpeedM1(0);
    setMotorSpeedM2(0);

    // Timer1
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11);
    OCR1A = twoMhz / maxSteps;
    TCNT1 = 0;
    TIMSK1 |= (1 << OCIE1A);

    cfAngleRadians = mpu.getAccelerationY() * a2g; // Init to accel estimate.
    motorSpeed = 0.0;

    angleITerm = 0.0;
    angleLastInput = 0.0;

//    // Parameters for short stack.
//    Kbp = 0.7; Kbd = 0.5;
//    Kap = 0.028; Kai = 0.000028; Kad = 0.55;
//    balanceOffset = -2.6;

//    // Parameters for short stack with long battery.
//    Kbp = 0.9; Kbd = 0.5;
//    Kap = 0.03; Kai = 0.000028; Kad = 0.4;
//    balanceOffset = 0;

    // Parameters for tall stack.
    Kbp = 3.0; Kbd = 1.0;
//    Kap = 0.0028; Kai = 0.000028; Kad = 0.028;
    Kap = 0.02; Kai = 0.00001; Kad = 0.01;
    balanceOffset = 0;

    pinMode(buzzer, OUTPUT);
    tone(buzzer, 200, 50);
    nextSampleTime = micros();
    lastForwardBackwardTime = nextSampleTime;
    lastLeftRightTime = nextSampleTime;
    lastForwardBackwardValue = LOW;
    lastLeftRightValue = LOW;
}

void initializeMPU() {
    mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    mpu.setFullScaleGyroRange(gyroRange);
    mpu.setFullScaleAccelRange(accelRange);
    mpu.setDLPFMode(MPU6050_DLPF_BW_10);
    mpu.setRate((1000/updatesPerSec)-1);
    mpu.setSleepEnabled(false);
}


int count = 0;


void loop() {
    rcUpdate();
    if (nextSample()) {

        int direction = map(lowPassLeftRight, 1300, 1700, 90, -90);
        float desiredSpeed = map(lowPassForwardBackward, 1000, 2000, -360, 360);

        if ((count++%20) == 0) {
            Serial.print(direction);
            Serial.print(" ");
            Serial.println(desiredSpeed);
        }

        // Get cfAngle and rotation from IMU.
        float cfAngle;
        float rotation;
        getCFAngleAndRotation(&cfAngle, &rotation);

        double speed = (stepsToRotationsSec(m1Speed)+
                        stepsToRotationsSec(m2Speed))/2;

        // More positive tends forward, negative tends backwards.
        float errorAngle = anglePID(desiredSpeed, speed) -
            cfAngle + balanceOffset;
        //        errorAngle = -cfAngle + balanceOffset;

        rcUpdate();

        // Basic PD controller with errorAngle and rotation as inputs. Since
        // the rotation is directly from the gyro it should be a more
        // accurate measurement of the change in angle than any calcuation
        // we would make.
        //
        // We don't want to integrate the error (the 'I' in PID) because
        // when we are moving we will correctly have a non-zero errorAngle
        // for an extended period of time.
        motorSpeed = clip(motorSpeed + Kbp*errorAngle - Kbd*rotation,
                          maxMotorSpeed);

        // Positive motor speeds go forward.
        if ((cfAngle > -stopMotorAngle) && (cfAngle < stopMotorAngle)) {
            setMotorSpeedM1(-motorSpeed + direction);
            setMotorSpeedM2(-motorSpeed - direction);
            digitalWrite(motorEnable, LOW);
        } else {
            digitalWrite(motorEnable, HIGH);
            setMotorSpeedM1(0);
            setMotorSpeedM2(0);
            motorSpeed = 0;
            angleITerm = 0.0;
        }
    }
}

void rcUpdate() {
    unsigned long now = micros();
    unsigned long fbTime = now - lastForwardBackwardTime;
    unsigned long lrTime = now - lastLeftRightTime;
    if ((fbTime > rcTimeOut) || (lrTime > rcTimeOut)) {
        lowPassLeftRight = 1500;
        lowPassForwardBackward = 1500;
        lastLeftRightValue = LOW;
        lastForwardBackwardValue = LOW;
    }
    int current = digitalRead(rcLeftRight);
    if (current != lastLeftRightValue) {
        if (current == LOW) {
            lowPassLeftRight =
                rcLowPassConst*lowPassLeftRight + (1-rcLowPassConst)*lrTime;
        }
        lastLeftRightValue = current;
        lastLeftRightTime = now;
    }
    current = digitalRead(rcForwardBackward);
    if (current != lastForwardBackwardValue) {
        if (current == LOW) {
            lowPassForwardBackward =
                rcLowPassConst*lowPassForwardBackward +
                (1-rcLowPassConst)*fbTime;
        }
        lastForwardBackwardValue = current;
        lastForwardBackwardTime = now;
    }
}


// Return 'cfAngle' and 'rotation' in degrees.
//   cfAngle  has a positive value when angled forward
//   rotation has a positive value when pitching forward
void getCFAngleAndRotation(float *cfAngle, float *rotation) {
    // The IMU's Y axis is the one on Doris that is 0 if Doris is exactly
    // vertical and goes +/- when she tilts. Use calculated conversions to
    // get these values to G and radians/sec.
    float a = -mpu.getAccelerationX() * a2g;
    rcUpdate();
    float r = mpu.getRotationY() * r2rs;
    rcUpdate();

    // Update complementary filter in radians. Technically, we should use
    // asin() on the acceleration to get the angle, however for small angles
    // in radians: sin(angle) ~ angle. Since we hope to stay close to
    // veritical, we can use this engineering approximation.
    cfAngleRadians = Kf*(cfAngleRadians + r/updatesPerSec) + (1.0-Kf)*a;

    // Now convert our radian values into degrees. We use degrees internally
    // because it makes it easier to understand values during debugging. We
    // could have stayed in radians for code simplicity.
    *cfAngle = cfAngleRadians*rad2deg;
    *rotation = r*rad2deg;
}


static bool nextSample() {
    // ***** WARNING ***** By using microseconds we wrap in 70 minutes from
    // start. We need to add some windowing code to allow the wrap to occur
    // without glitches.
    //
    // When using millis() I noticed an occasional start delay of 1 ms or,
    // more importantly, a 10% error.  By using micros() I measured a max
    // start delay of less than 100us or 1% error.
    now = micros();
    if (nextSampleTime < now) {
//        if ((now-nextSampleTime) > 100) {
//            tone(buzzer, 262, 5);
//        }
        nextSampleTime += 1000000/updatesPerSec;
        return true;
    }
    return false;
}

static float clip(float val, float maxVal) {
    if (val < -maxVal) {
        val = -maxVal;
    } else if (val > maxVal) {
        val = maxVal;
    }
    return val;
}

static float anglePID(float setpoint, float input) {
    float error = setpoint - input;
    float anglePTerm = Kap * error;
    angleITerm = clip(angleITerm + (Kai * error), maxAngle);
    float angleDTerm = Kad * (angleLastInput - input);
    angleLastInput = input;
    return clip(anglePTerm + angleITerm + angleDTerm, maxAngle);
}

static int stepsToRotationsSec(int16_t steps) {
    return (int)((float)steps/stepsInRotation*360);
}

static void setMotorSpeedM1(int16_t newSpeed) {
    newSpeed = clip(newSpeed, maxMotorSpeed);
    int16_t desiredSpeed = (int16_t)(((float)newSpeed/360.0)*stepsInRotation);
    if ((m1Speed - desiredSpeed) > maxStepperAcceleration) {
        m1Speed -= maxStepperAcceleration;
    } else if ((m1Speed - desiredSpeed) < -maxStepperAcceleration) {
        m1Speed += maxStepperAcceleration;
    } else {
        m1Speed = desiredSpeed;
    }
    if (m1Speed == 0) {
        m1Running = 0;
    } else {
        int16_t newValue = maxSteps / (int)(m1Speed*use16steps);
        if (newValue != m1ResetValue) {
            // int16_t newCounter = newValue * ((float)m1Counter / m1ResetValue);
            if (newValue > 0) {
                digitalWrite(leftDir, HIGH);
                m1ResetValue = newValue;
            } else {
                digitalWrite(leftDir, LOW);
                m1ResetValue = -newValue;
            }
            // m1Counter = 0;
            // m1Counter = newCounter;
            if (m1Counter > m1ResetValue) {
                m1Counter = m1ResetValue;
            }
            m1Running = 1;
        }
    }
}

static void setMotorSpeedM2(int16_t newSpeed) {
    newSpeed = clip(newSpeed, maxMotorSpeed);
    int16_t desiredSpeed = (int16_t)(((float)newSpeed/360.0)*stepsInRotation);
    if ((m2Speed - desiredSpeed) > maxStepperAcceleration) {
        m2Speed -= maxStepperAcceleration;
    } else if ((m2Speed - desiredSpeed) < -maxStepperAcceleration) {
        m2Speed += maxStepperAcceleration;
    } else {
        m2Speed = desiredSpeed;
    }
    if (m2Speed == 0) {
        m2Running = 0;
    } else {
        int16_t newValue = maxSteps / (int)(m2Speed*use16steps);
        if (newValue != m2ResetValue) {
            // int16_t newCounter = newValue * ((float)m2Counter / m2ResetValue);
            if (newValue > 0) {
                digitalWrite(rightDir, LOW);
                m2ResetValue = newValue;
            } else {
                digitalWrite(rightDir, HIGH);
                m2ResetValue = -newValue;
            }
            // m2Counter = 0;
            // m2Counter = newCounter;
            if (m2Counter > m2ResetValue) {
                m2Counter = m2ResetValue;
            }
            m2Running = 1;
        }
    }
}

// TIMER 1 : Stepper speed control
ISR(TIMER1_COMPA_vect) {
    if (m1Running) {
        if (--m1Counter <= 0) {
            m1Counter = m1ResetValue;
            digitalWrite(leftStep, HIGH);
            delay_1us();
            digitalWrite(leftStep, LOW);
        }
    }
    if (m2Running) {
        if (--m2Counter <= 0) {
            m2Counter = m2ResetValue;
            digitalWrite(rightStep, HIGH);
            delay_1us();
            digitalWrite(rightStep, LOW);
        }
    }
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
