// This is V2's first balancing code. I truly stood on the shoulder of
// giants as I created this. There is a ton of great material on the web
// about IMUs, Complementary Filters, PIDs, and stepper motor drivers.
//
// This version uses the Arduino Leonardo's two 16-bit timers to generate
// the necessary stepper pulses. That works great for a two-wheel
// segway-style balancer, but it won't work for a three-motor ballbot. The
// Leonardo ony has two 16-bit timers. So I will need to find a different
// approach to driver the stepper motor.

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

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

#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

const int      stepsInRotation = 200;       // Number of steps per rotation.
const int      maxMotorSpeed = 4*360;       // Maximum motor speed in degrees/sec.
const uint16_t slowestSpeed = 65535;        // Slowest speed for timer.
const int      use16steps = 16;             // Multiplier for 16 microsteps.
const long     twoMhz = 2000000;            // Stepper 2Mhz constant.

// Maximum stepper acceleration which is based on the update frequency.
const int      maxStepperAcceleration = 1600/updatesPerSec;

static int16_t m1Speed;         // Motor speed.
static int8_t  m1Running;       // True if motors running.
static int16_t m2Speed;         // Motor speed.
static int8_t  m2Running;       // True if motors running.

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
    m1Speed = 0;
    m2Speed = 0;
    m1Running = 0;
    m2Running = 0;
    pinMode(4, OUTPUT);         // Enable pin set to output.
    pinMode(7, OUTPUT);         // Motor 1 step porte,6
    pinMode(8, OUTPUT);         // Motor 1 direction portb,4
    pinMode(12, OUTPUT);        // Motor 2 step portd,6
    pinMode(5, OUTPUT);         // Motor 2 direction portc,6
    digitalWrite(4, HIGH);      // Enable pin set to disable motors

    // Timer1
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11);
    OCR1A = slowestSpeed;
    TCNT1 = 0;

    // Timer3
    TCCR3A = 0;
    TCCR3B = (1 << WGM32) | (1 << CS31);
    OCR3A = slowestSpeed;
    TCNT3 = 0;

    // Enable TIMERs interrupts
    TIMSK1 |= (1 << OCIE1A);
    TIMSK3 |= (1 << OCIE1A);

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
    if (nextSample()) {
        int direction = 0;      // Deal with this later.
        float desiredSpeed = 0.0/360;

        // Get cfAngle and rotation from IMU.
        float cfAngle;
        float rotation;
        getCFAngleAndRotation(&cfAngle, &rotation);

        double speed = (stepsToRotationsSec(m1Speed)+
                        stepsToRotationsSec(m2Speed))/2;

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
        int16_t motor1 = motorSpeed + direction;
        int16_t motor2 = motorSpeed - direction;
        if ((cfAngle > -stopMotorAngle) && (cfAngle < stopMotorAngle)) {
            setMotorSpeedM1(motor1);
            setMotorSpeedM2(motor2);
            digitalWrite(4, LOW);
        } else {
            digitalWrite(4, HIGH);
            setMotorSpeedM1(0);
            setMotorSpeedM2(0);
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

static int stepsToRotationsSec(int16_t steps) {
    return (int)((float)steps/stepsInRotation*360);
}

static void setMotorSpeedM1(int16_t newSpeed) {
    long timer_period;

    // Clip to our maximum speed.
    newSpeed = clip(newSpeed, maxMotorSpeed);

    int16_t desiredSpeed = (int16_t)(((float)newSpeed/360.0)*stepsInRotation);

    // Limit the maximum acceleration of the motors
    if ((m1Speed - desiredSpeed) > maxStepperAcceleration) {
        m1Speed -= maxStepperAcceleration;
    } else if ((m1Speed - desiredSpeed) < -maxStepperAcceleration) {
        m1Speed += maxStepperAcceleration;
    } else {
        m1Speed = desiredSpeed;
    }

    int16_t speed = m1Speed * use16steps;
    if (speed == 0) {
        timer_period = slowestSpeed;
        m1Running = 0;
    } else if (speed > 0) {
        timer_period = twoMhz / speed;
        m1Running = 1;
        SET(PORTB, 4);          // Set direction forward.
    } else {
        timer_period = twoMhz / -speed;
        m1Running = 1;
        CLR(PORTB, 4);          // Set direction backwards.
    }
    if (timer_period > slowestSpeed) {
        timer_period = slowestSpeed;
    }
    OCR1A = timer_period;
    // We should reset the timer.
    if (TCNT1 > OCR1A) {
        TCNT1 = 0;
    }
}

static void setMotorSpeedM2(int16_t newSpeed) {
    long timer_period;

    // Clip to our maximum speed.
    newSpeed = clip(newSpeed, maxMotorSpeed);

    int16_t desiredSpeed = (int16_t)(((float)newSpeed/360.0)*stepsInRotation);

    // Limit the maximum acceleration of the motors
    if ((m2Speed - desiredSpeed) > maxStepperAcceleration) {
        m2Speed -= maxStepperAcceleration;
    } else if ((m2Speed - desiredSpeed) < -maxStepperAcceleration) {
        m2Speed += maxStepperAcceleration;
    } else {
        m2Speed = desiredSpeed;
    }
    int16_t speed = m2Speed * use16steps;
    if (speed == 0) {
        timer_period = slowestSpeed;
        m2Running = 0;
    } else if (speed > 0) {
        timer_period = twoMhz / speed;
        m2Running = 1;
        CLR(PORTC, 6);          // Set direction forward.
    } else {
        timer_period = twoMhz / -speed;
        m2Running = 1;
        SET(PORTC, 6);          // Set direction backwards.
    }
    if (timer_period > slowestSpeed) {
        timer_period = slowestSpeed;
    }
    OCR3A = timer_period;
    // We should reset the timer.
    if (TCNT3 > OCR3A) {
        TCNT3 = 0;
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

// TIMER 1 : Stepper motor1 speed control
ISR(TIMER1_COMPA_vect) {
    if (m1Running) {
        SET(PORTE, 6);
        delay_1us();
        CLR(PORTE, 6);
    }
}

// TIMER 3 : Stepper motor2 speed control
ISR(TIMER3_COMPA_vect) {
    if (m2Running) {
        SET(PORTD, 6);
        delay_1us();
        CLR(PORTD, 6);
    }
}
