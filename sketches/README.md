# Sketches

This directory holds Arduino code.

[v2/balance](https://github.com/SteveGeyer/Doris/tree/master/sketches/v2/balance) is the first version of the code I got working. This version uses the Arduino Leonardo's two 16-bit timers to generate the necessary stepper pulses. That works great for a two-wheel segway-style balancer, but it won't work for a three-motor ballbot. The Leonardo ony has two 16-bit timers. So I will need to find a different approach to drive the stepper motor.

[v2/nointerrupts](https://github.com/SteveGeyer/Doris/tree/master/sketches/v2/nointerrupts) does not use the 16-bit timer interrupts at all. Instead it steps each motor by polling against the elasped time and then pulsing the stepper. This code seems to balance well as well as the original code, but the stepper motors give off a more musical sounds. I have no idea why they sound different.
