// My first attempt at a v3 ballbot using stepping motors.
//
// Unfortunately, I discovered that the stepping motors I am using cannot
// drive the omniwheels with enough torque when the omniwheel are being
// forced to slip sideways. I attempted this with two different omniwheels
// and both failed. I believe my next step is to use geared DC motors, but
// that will require redesign.
//
// Even thought this code was not able to be made to work I am checking it
// in as a starting place for some future ballbot. Be warned that it is in
// some intermediate debugging state.

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
const float   stopMotorAngle = 10; // In degrees.
const int     updatesPerSec = 100;
const uint8_t gyroRange     = MPU6050_GYRO_FS_2000;
const uint8_t accelRange    = MPU6050_ACCEL_FS_2;
const float   switchToAccel = 0.5; // Secs before trusting accel.
const float   Kf            = switchToAccel/(switchToAccel+1.0/updatesPerSec);

const int      maxRotations = 4;                 // Maximum rotations/sec.
const int      stepsInRotation = 200;            // Number of steps per rotation.
const int      microSteps = 16;                  // Number of microsteps (must be 1,2,4,8,16).
const int      maxMotorSpeed = maxRotations*360; // Maximum motor speed in degrees/sec.
const uint16_t slowestSpeed = 65535;             // Slowest speed for timer.
const long     twoMhz = 2000000;                 // Stepper 2Mhz constant.

const int      maxSteps = maxRotations * stepsInRotation * microSteps;

// Maximum stepper acceleration which is based on the update frequency.
const int      maxStepperAcceleration = 1600/updatesPerSec;

static int16_t   mLSpeed;       // Motor speed,
volatile int8_t  mLRunning;     //  true if motors running,
volatile int16_t mLCounter;     //   interrupt counter,
volatile int16_t mLResetValue;  //    and interrupt reset value.

static int16_t   mRSpeed;       // Motor speed,
volatile int8_t  mRRunning;     //  true if motors running,
volatile int16_t mRCounter;     //   interrupt counter,
volatile int16_t mRResetValue;  //    and interrupt reset value.

static int16_t   mFSpeed;       // Motor speed,
volatile int8_t  mFRunning;     //  true if motors running,
volatile int16_t mFCounter;     //   interrupt counter,
volatile int16_t mFResetValue;  //    and interrupt reset value.

static MPU6050 mpu;             // Gyroscopes and accelerometers.
static float r2rs;              // Rotation int16 to radians/sec.
static float a2g;               // Acceleration int to G.
static float cfAngleRadiansX;   // Raw complementary filter angle in radians.
static float cfAngleRadiansY;   // Raw complementary filter angle in radians.

static float motorSpeedX;       // Speed to command motors.
static float motorSpeedY;       // Speed to command motors.

static float angleITerm;        // Angle PID iTerm.
static float angleLastInput;    // Angle PID last input (for dTerm).

static unsigned long nextSampleTime;

static float Kap;               // Speed to angle proportional.
static float Kai;               // Speed to angle integrative.
static float Kad;               // Speed to angle derivative.
static float Kbp;               // Balance proportional.
static float Kbd;               // Balance derivative.
static float balanceOffset;     // This is the balace angle for robot.

static void delay_1us();
static bool nextSample();
static float clip(float val, float maxVal);
static void setMotorSpeedML(int16_t tspeed);
static void setMotorSpeedMR(int16_t tspeed);

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
    pinMode(motorMS2, OUTPUT);
    pinMode(motorMS3, OUTPUT);
    digitalWrite(motorMS1, ((microSteps == 2) ||
                            (microSteps == 8) ||
                            (microSteps == 16)) ? HIGH : LOW);
    digitalWrite(motorMS2, ((microSteps == 4) ||
                            (microSteps == 8) ||
                            (microSteps == 16)) ? HIGH : LOW);
    digitalWrite(motorMS3,  (microSteps == 16) ? HIGH : LOW);

    // Setup stepper motor values and pins.
    pinMode(frontDir, OUTPUT);
    pinMode(frontStep, OUTPUT);


    pinMode(leftDir, OUTPUT);
    pinMode(leftStep, OUTPUT);
    mLSpeed = 0;
    mLRunning = 0;
    mLCounter = 0;
    mLResetValue = 0;

    pinMode(rightDir, OUTPUT);
    pinMode(rightStep, OUTPUT);
    mRSpeed = 0;
    mRRunning = 0;
    mRCounter = 0;
    mRResetValue = 0;

    pinMode(frontDir, OUTPUT);
    pinMode(frontStep, OUTPUT);
    mFSpeed = 0;
    mFRunning = 0;
    mFCounter = 0;
    mFResetValue = 0;

    setMotorSpeedML(0);
    setMotorSpeedMR(0);
    setMotorSpeedMF(0);

    // Timer1
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11);
    OCR1A = twoMhz / maxSteps;
    TCNT1 = 0;
    TIMSK1 |= (1 << OCIE1A);

    cfAngleRadiansX = mpu.getAccelerationX() * a2g; // Init to accel estimate.
    cfAngleRadiansY = mpu.getAccelerationY() * a2g; // Init to accel estimate.
    motorSpeedX = 0.0;
    motorSpeedY = 0.0;

    angleITerm = 0.0;
    angleLastInput = 0.0;

    // Parameters for short stack with long battery.
    // Kbp = 0.9; Kbd = 0.5;
    Kbp = 0.9; Kbd = 0.0;
    Kap = 0.03; Kai = 0.000028; Kad = 0.4;
    balanceOffset = 0;

    pinMode(buzzer, OUTPUT);
    tone(buzzer, 200, 50);
    nextSampleTime = micros();
}

void initializeMPU() {
    mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    mpu.setFullScaleGyroRange(gyroRange);
    mpu.setFullScaleAccelRange(accelRange);
    mpu.setDLPFMode(MPU6050_DLPF_BW_10);
    mpu.setRate((1000/updatesPerSec)-1);
    mpu.setSleepEnabled(false);
}

long count = 0;

void loop() {
    if (nextSample()) {
        float desiredSpeed = 0.0/360;

        // Get cfAngle and rotation from IMU.
        float cfAngleX, cfAngleY;
        float rotationX, rotationY;
        getCFAngleAndRotationX(&cfAngleX, &rotationX);
        getCFAngleAndRotationY(&cfAngleY, &rotationY);
        float cfAngle = sqrt(cfAngleX*cfAngleX + cfAngleY*cfAngleY);

        //         double speed = (stepsToRotationsSec(mLSpeed)+
        //                         stepsToRotationsSec(mRSpeed))/2;
        //         // More positive tends forward, negative tends backwards.
        //         float errorAngle = anglePID(desiredSpeed, speed) - cfAngle + balanceOffset;
        float errorAngleX = -cfAngleX;
        float errorAngleY = -cfAngleY;

        // // Basic PD controller with errorAngle and rotation as inputs. Since
        // // the rotation is directly from the gyro it should be a more accurate
        // // measurement of the change in angle than any calcuation we would make.
        // //
        // // We don't want to integrate the error (the 'I' in PID) because
        // // when we are moving we will correctly have a non-zero errorAngle
        // // for an extended period of time.
        motorSpeedX += Kbp*errorAngleX - Kbd*rotationX;
        motorSpeedX = clip(motorSpeedX, maxMotorSpeed);

        motorSpeedY += Kbp*errorAngleY - Kbd*rotationY;
        motorSpeedY = clip(motorSpeedY, maxMotorSpeed);

        // motorSpeedX = errorAngleX; // TEMP HACK
        // motorSpeedY = errorAngleY; // TEMP HACK

        float theta = atan2(motorSpeedY, motorSpeedX)*rad2deg;
        float motorSpeed = sqrt(motorSpeedX*motorSpeedX + motorSpeedY*motorSpeedY);

        theta = 0;
        motorSpeed = 180;

        // Positive motor speeds go forward.
        if ((cfAngle > -stopMotorAngle) && (cfAngle < stopMotorAngle)) {
            float thetaRads = theta*deg2rad;
            int16_t mf = motorSpeed * sin(thetaRads + 3*(2*PI/6));
            int16_t ml = motorSpeed * sin(thetaRads + 1*(2*PI/6));
            int16_t mr = motorSpeed * sin(thetaRads + 5*(2*PI/6));
            // if ((count++ % 20) == 0) {
            //     Serial.print(cfAngle);
            //     Serial.print(" | ");
            //     Serial.print(motorSpeedX);
            //     Serial.print(" ");
            //     Serial.print(motorSpeedY);
            //     Serial.print(" => ");
            //     Serial.print(theta);
            //     Serial.print(" ");
            //     Serial.print(motorSpeed);
            //     Serial.print(" | l:");
            //     Serial.print(ml);
            //     Serial.print(" f:");
            //     Serial.print(mf);
            //     Serial.print(" r:");
            //     Serial.println(mr);
            // }
            setMotorSpeedMF(mf);
            setMotorSpeedML(ml);
            setMotorSpeedMR(mr);
            digitalWrite(motorEnable, LOW);
        } else {
            Serial.println("STOP");
            digitalWrite(motorEnable, HIGH);
            setMotorSpeedMF(0);
            setMotorSpeedML(0);
            setMotorSpeedMR(0);
            motorSpeedX = 0;
            motorSpeedY = 0;
            // angleITerm = 0.0;
        }
    }
}

// Return 'cfAngle' and 'rotation' in degrees.
//   cfAngle  has a positive value when angled forward
//   rotation has a positive value when pitching forward
void getCFAngleAndRotationX(float *cfAngle, float *rotation) {
    // The IMU's Y axis is the one on Doris that is 0 if Doris is exactly
    // vertical and goes +/- when she tilts. Use calculated conversions to
    // get these values to G and radians/sec.
    float a = -mpu.getAccelerationX() * a2g;
    float r = mpu.getRotationY() * r2rs;

    // Update complementary filter in radians. Technically, we should use
    // asin() on the acceleration to get the angle, however for small angles
    // in radians: sin(angle) ~ angle. Since we hope to stay close to
    // veritical, we can use this engineering approximation.
    cfAngleRadiansX = Kf*(cfAngleRadiansX + r/updatesPerSec) + (1.0-Kf)*a;

    // Now convert our radian values into degrees. We use degrees internally
    // because it makes it easier to understand values during debugging. We
    // could have stayed in radians for code simplicity.
    *cfAngle = cfAngleRadiansX*rad2deg;
    *rotation = r*rad2deg;
}

// Return 'cfAngle' and 'rotation' in degrees.
//   cfAngle  has a positive value when angled left.
//   rotation has a positive value when pitching left.
void getCFAngleAndRotationY(float *cfAngle, float *rotation) {
    // The IMU's Y axis is the one on Doris that is 0 if Doris is exactly
    // vertical and goes +/- when she tilts. Use calculated conversions to
    // get these values to G and radians/sec.
    float a = -mpu.getAccelerationY() * a2g;
    float r = -mpu.getRotationX() * r2rs;

    // Update complementary filter in radians. Technically, we should use
    // asin() on the acceleration to get the angle, however for small angles
    // in radians: sin(angle) ~ angle. Since we hope to stay close to
    // veritical, we can use this engineering approximation.
    cfAngleRadiansY = Kf*(cfAngleRadiansY + r/updatesPerSec) + (1.0-Kf)*a;

    // Now convert our radian values into degrees. We use degrees internally
    // because it makes it easier to understand values during debugging. We
    // could have stayed in radians for code simplicity.
    *cfAngle = cfAngleRadiansY*rad2deg;
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
    unsigned long now = micros();
    if (nextSampleTime < now) {
        if ((now-nextSampleTime) > 200) {
            tone(buzzer, 262, 5);
        }
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
    float output = clip(anglePTerm + angleITerm + angleDTerm, maxAngle);
    output = -output;  // This guy needs to be inverted.
    return output;
}

static int stepsToRotationsSec(int16_t steps) {
    return (int)((float)steps/stepsInRotation*360);
}

static void setMotorSpeedML(int16_t newSpeed) {
    newSpeed = clip(newSpeed, maxMotorSpeed);
    int16_t desiredSpeed = (int16_t)(((float)newSpeed/360.0)*stepsInRotation);
    if ((mLSpeed - desiredSpeed) > maxStepperAcceleration) {
        mLSpeed -= maxStepperAcceleration;
    } else if ((mLSpeed - desiredSpeed) < -maxStepperAcceleration) {
        mLSpeed += maxStepperAcceleration;
    } else {
        mLSpeed = desiredSpeed;
    }
    if (mLSpeed == 0) {
        mLRunning = 0;
    } else {
        int16_t newValue = maxSteps / (int)(mLSpeed*microSteps);
        if (newValue != mLResetValue) {
            // int16_t newCounter = newValue * ((float)mLCounter / mLResetValue);
            if (newValue > 0) {
                digitalWrite(leftDir, HIGH);
                mLResetValue = newValue;
            } else {
                digitalWrite(leftDir, LOW);
                mLResetValue = -newValue;
            }
            // mLCounter = 0;
            // mLCounter = newCounter;
            if (mLCounter > mLResetValue) {
                mLCounter = mLResetValue;
            }
            mLRunning = 1;
        }
    }
}

static void setMotorSpeedMR(int16_t newSpeed) {
    newSpeed = clip(newSpeed, maxMotorSpeed);
    int16_t desiredSpeed = (int16_t)(((float)newSpeed/360.0)*stepsInRotation);
    if ((mRSpeed - desiredSpeed) > maxStepperAcceleration) {
        mRSpeed -= maxStepperAcceleration;
    } else if ((mRSpeed - desiredSpeed) < -maxStepperAcceleration) {
        mRSpeed += maxStepperAcceleration;
    } else {
        mRSpeed = desiredSpeed;
    }
    if (mRSpeed == 0) {
        mRRunning = 0;
    } else {
        int16_t newValue = maxSteps / (int)(mRSpeed*microSteps);
        if (newValue != mRResetValue) {
            // int16_t newCounter = newValue * ((float)mRCounter / mRResetValue);
            if (newValue > 0) {
                digitalWrite(rightDir, HIGH);
                mRResetValue = newValue;
            } else {
                digitalWrite(rightDir, LOW);
                mRResetValue = -newValue;
            }
            // mRCounter = 0;
            // mRCounter = newCounter;
            if (mRCounter > mRResetValue) {
                mRCounter = mRResetValue;
            }
            mRRunning = 1;
        }
    }
}

static void setMotorSpeedMF(int16_t newSpeed) {
    newSpeed = clip(newSpeed, maxMotorSpeed);
    int16_t desiredSpeed = (int16_t)(((float)newSpeed/360.0)*stepsInRotation);
    if ((mFSpeed - desiredSpeed) > maxStepperAcceleration) {
        mFSpeed -= maxStepperAcceleration;
    } else if ((mFSpeed - desiredSpeed) < -maxStepperAcceleration) {
        mFSpeed += maxStepperAcceleration;
    } else {
        mFSpeed = desiredSpeed;
    }
    if (mFSpeed == 0) {
        mFRunning = 0;
    } else {
        int16_t newValue = maxSteps / (int)(mFSpeed*microSteps);
        if (newValue != mFResetValue) {
            // int16_t newCounter = newValue * ((float)mFCounter / mFResetValue);
            if (newValue > 0) {
                digitalWrite(frontDir, HIGH);
                mFResetValue = newValue;
            } else {
                digitalWrite(frontDir, LOW);
                mFResetValue = -newValue;
            }
            // mFCounter = 0;
            // mFCounter = newCounter;
            if (mFCounter > mFResetValue) {
                mFCounter = mFResetValue;
            }
            mFRunning = 1;
        }
    }
}


// TIMER 1 : Stepper speed control
ISR(TIMER1_COMPA_vect) {
    if (mLRunning) {
        if (--mLCounter <= 0) {
            mLCounter = mLResetValue;
            digitalWrite(leftStep, HIGH);
            delay_1us();
            digitalWrite(leftStep, LOW);
        }
    }
    if (mRRunning) {
        if (--mRCounter <= 0) {
            mRCounter = mRResetValue;
            digitalWrite(rightStep, HIGH);
            delay_1us();
            digitalWrite(rightStep, LOW);
        }
    }
    if (mFRunning) {
        if (--mFCounter <= 0) {
            mFCounter = mFResetValue;
            digitalWrite(frontStep, HIGH);
            delay_1us();
            digitalWrite(frontStep, LOW);
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
