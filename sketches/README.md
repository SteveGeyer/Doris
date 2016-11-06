# Sketches

This directory holds Arduino code.


[v2/balance](https://github.com/SteveGeyer/Doris/tree/master/sketches/v2/balance) is the first version of the code I got working. This version uses the Arduino Leonardo's two 16-bit timers to generate the necessary stepper pulses. That works great for a two-wheel segway-style balancer, but it won't work for a three-motor ballbot. The Leonardo ony has two 16-bit timers. So I will need to find a different approach to drive the stepper motor.

[v2/nointerrupts](https://github.com/SteveGeyer/Doris/tree/master/sketches/v2/nointerrupts) does not use the 16-bit timer interrupts at all. Instead it steps each motor by polling against the elasped time and then pulsing the stepper. This code seems to balance well as well as the original code, but the stepper motors give off a more musical sounds. I have no idea why they sound different.

[v3/nointerrupts](https://github.com/SteveGeyer/Doris/tree/master/sketches/v3/nointerrupts) is the same code as the V2 version modified to use an Arduino Uno and the custom PC board's I/O pin layout. I have noticed a little instablity in the V3 balance. I will be working to resolve this.

[v3/oneinterrupt](https://github.com/SteveGeyer/Doris/tree/master/sketches/v3/interrupt) uses a single interrupt to handle multiple motors stepping function. It is works great with no instability in balance. In future steps my plan is to use this version for the ballbot.

[v3/ballbot](https://github.com/SteveGeyer/Doris/tree/master/sketches/v3/ballbot) uses a single interrupt to handle multiple motors stepping function. Unfortunately, I discovered that the stepping motors I am using cannot drive the omniwheels with enough torque when the omniwheel are being forced to slip sideways. I attempted this with two different omniwheels and both failed. I believe my next step is to use geared DC motors, but that will require redesign. Even thought this code was not able to be made to work I am checking it in as a starting place for some future ballbot. Be warned that it is in some intermediate debugging state.

[MPU6050_calibration](https://github.com/SteveGeyer/Doris/tree/master/sketches/MPU6050_calibration) is the sketch used to dervive the calibration parameters for the MPU6050. These parameters and currently wired into the code. This sketch was written by Luis Ródenas <luisrodenaslorda@gmail.com>.
