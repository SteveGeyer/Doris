// Unlike "balance.ino", this balancing code that does not use the 16-bit
// timer interrupts at all. Instead it steps each motor by polling against
// the elasped time and then pulsing the stepper.
//
// This code seems to balance well as well as the original code, but the
// stepper motors give off a more musical sounds. I have no idea why they
// sound different.

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "motor.h"

// enable: D4 low
// m1:
//   step: PE6 -- D7  -- PD7
//   dir:  PB4 -- D8  -- PB0
// m2:
//   step: PD6 -- D12 -- PB4
//   dir:  PC6 -- D5  -- PD5

// Physical constants.
const float rad2deg = 180/PI;
const float deg2rad = 1/rad2deg;

// Parameters gotten from MPU6050_calibration
const long xAccelOff = -49;
const long yAccelOff = -1354;
const long zAccelOff = 838;
const long xGyroOff = 117;
const long yGyroOff = -48;
const long zGyroOff = 20;

const float   maxAngle = 10;       // In degrees.
const float   stopMotorAngle = 15; // In degrees.
const int     updatesPerSec = 200;
const uint8_t gyroRange     = MPU6050_GYRO_FS_2000;
const uint8_t accelRange    = MPU6050_ACCEL_FS_2;
const float   switchToAccel = 0.5; // Secs before trusting accel.
const float   Kf            = switchToAccel/(switchToAccel+1.0/updatesPerSec);

static Motor motor1(7, 8, updatesPerSec, true);
static Motor motor2(12, 5, updatesPerSec, false);

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

static void delay_1us();
static bool nextSample();
static float clip(float val, float maxVal);
static void setMotorSpeedM1(int16_t tspeed);
static void setMotorSpeedM2(int16_t tspeed);

void setup() {
    Serial.begin(115200);

    Wire.begin();
    TWBR = 24;                  // 400kHz I2C clock (200kHz if CPU is 8MHz).

    mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    mpu.setFullScaleGyroRange(gyroRange);
    mpu.setFullScaleAccelRange(accelRange);
    mpu.setDLPFMode(MPU6050_DLPF_BW_10);
    mpu.setRate((1000/updatesPerSec)-1);
    mpu.setSleepEnabled(false);

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
    pinMode(4, OUTPUT);         // Enable pin set to output.
    digitalWrite(4, LOW);
    motor1.SetSpeed(0);
    motor2.SetSpeed(0);

    cfAngleRadians = mpu.getAccelerationY() * a2g; // Init to accel estimate.
    motorSpeed = 0.0;

    angleITerm = 0.0;
    angleLastInput = 0.0;

//    // Parameters for short stack.
//    Kbp = 0.7; Kbd = 0.5;
//    Kap = 0.028; Kai = 0.000028; Kad = 0.55;
//    balanceOffset = -2.6;

    // Parameters for short stack with long battery.
    Kbp = 0.9; Kbd = 0.5;
    Kap = 0.03; Kai = 0.000028; Kad = 0.4;
    balanceOffset = 0;

    // Parameters for tall stack.
//    Kbp = 3.0; Kbd = 1.0;
//    Kap = 0.0028; Kai = 0.000028; Kad = 0.028;
//    balanceOffset = -2.3;

//    while (!Serial); // wait for Leonardo enumeration, others continue immediately
//    Serial.println("Started");
    nextSampleTime = micros();
}

void loop() {
    motor1.Run();
    motor2.Run();
    if (nextSample()) {
        int direction = 0;      // Deal with this later.
        float desiredSpeed = 0.0/360;

        // Get cfAngle and rotation from IMU.
        float cfAngle;
        float rotation;
        getCFAngleAndRotation(&cfAngle, &rotation);

        double speed = (motor1.CurrentSpeed()+motor2.CurrentSpeed())/2;

        // More positive tends forward, negative tends backwards.
        // float errorAngle = -cfAngle + balanceOffset + 0.0;
        float errorAngle = anglePID(desiredSpeed, speed) - cfAngle + balanceOffset;
//        errorAngle = -cfAngle + balanceOffset;

        // Basic PD controller with errorAngle and rotation as inputs. Since
        // the rotation is directly from the gyro it should be a more accurate
        // measurement of the change in angle than any calcuation we would make.
        //
        // We don't want to integrate the error (the 'I' in PID) because
        // when we are moving we will correctly have a non-zero errorAngle
        // for an extended period of time.
        motorSpeed += clip(Kbp*errorAngle - Kbd*rotation, maxMotorSpeed);

        // Positive motor speeds go forward.
        if ((cfAngle > -stopMotorAngle) && (cfAngle < stopMotorAngle)) {
            motor1.SetSpeed(motorSpeed + direction);
            motor2.SetSpeed(motorSpeed - direction);
            digitalWrite(4, LOW);
        } else {
            digitalWrite(4, HIGH);
            motor1.SetSpeed(0);
            motor2.SetSpeed(0);
            motorSpeed = 0;
            angleITerm = 0.0;
        }
    }
}

// Return 'cfAngle' and 'rotation' in degrees.
//   cfAngle  has a positive value when angled forward
//   rotation has a positive value when pitching forward
void getCFAngleAndRotation(float *cfAngle, float *rotation) {
    // The IMU's Y axis is the one on Doris that is 0 if Doris is exactly
    // vertical and goes +/- when she tilts. Use calculated conversions to
    // get these values to G and radians/sec.
    float a = mpu.getAccelerationY() * a2g;
    float r = mpu.getRotationX() * r2rs;

    // Update complementary filter in radians. Technically, we should use
    // asin() on the acceleration to get the angle, however for small angles
    // in radians: sin(angle) ~ angle. Since we hope to stay close to
    // veritical, we can use this engineering approximation.
    cfAngleRadians = Kf*(cfAngleRadians + r/updatesPerSec) + (1.0-Kf)*a;

    // Now convert our radian values into degrees. We use degrees internally
    // because it makes it easier to understand values during debugging. We
    // could have stayed in radians for code simplicity.
    *cfAngle = -cfAngleRadians*rad2deg;
    *rotation = -r*rad2deg;
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
