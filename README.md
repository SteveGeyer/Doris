# Doris

Doris represents a series of balancing robots I have been working on over the last several years. This repository is where I am placing her design files. Over time I plan to have complete instructions on how to replicate my results.

Doris started her life as a ballbot. As I struggled to get her to balance I decided that starting with a ballbot was too audacious and maybe I should fall back to a two-wheel segway-style robot. Given her modular design it was easy to modified her for two wheels. After a lot of experimentation and learning I have successfully gotten Doris to balance solidly. Afterwards, I gave [this](https://github.com/SteveGeyer/Doris/blob/master/docs/BalancingRobotTalk.pdf) talk on what I have learned.

![V2&V3 Robot](https://github.com/SteveGeyer/Doris/blob/master/docs/TwoGenerations.jpg "V2&V3 Robot")

There have been several generations of Doris. V1 is now history. The left hand robot above is the currently working V2 design. It is a two wheel design based on stepper motors, an Arduino Leonardo, a MPU6050 IMU and A4988 stepper drivers. On the right is the V3 prototype in active development. It extends the V2 design by using an Arduino Uno along with a custom PC board for the motor drivers and IMU. It also supports both two wheel and three wheel ballbot designs. The picture shows the ballbot version. However, on further testing I discovered that the stepping motors did not have enough torque to push the omniwheels when they needed to slide. It looks like geared DC motors woudld be a better choice in future designs. I am leaving the V3 ballbot designs in this repository in the hopes that it will be the basis for a later design.

This repository holds both the V2 and V3 designs.

# 3D Models

Doris has a modular design using three 5mm carbon fiber rods as a backbone and then 3D printing mount components that sit between these carbon fiber rods. These mounts are custom designed to their purpose and they can be shared between the V2 and V3 designs. In the picture above of the V2 robot balancing you can see a topMount capping the carbon fiber rods, two battery mounts, an arduino mount and the two wheel motor mount. Below is an exploded view of all the 3D printed parts for both the two-wheel segway design and the three wheel ballbot design:

![Exploded parts](https://github.com/SteveGeyer/Doris/blob/master/docs/ExplodedView.png "Exploded parts")

The OpenSCAD models for all these parts can be found in the [models](https://github.com/SteveGeyer/Doris/tree/master/models) directory.

# Electrical Design

The V2 version of Doris has a hand-built protoboard mounted on an Arduino Leonardo. The schematic for this protoboard can be found [here](https://github.com/SteveGeyer/Doris/blob/master/schematics/v2/TwoWheelDorisV2.pdf) in the [schematic](https://github.com/SteveGeyer/Doris/tree/master/schematics) directory. Here is a picture of the board:

![V2 Protoboard](https://github.com/SteveGeyer/Doris/blob/master/docs/ProtoboardV2.png "V2 Protoboard")

The V3 version will have a custom PC board design to support either two or three stepper motors. The first revision of this board had some issues, but with small modifications it was good enough to balance on two wheels. I will add the Eagle files to this repository after I submit the next version of the board for fab.

# Software

The software driving Doris can be found in the [sketches](https://github.com/SteveGeyer/Doris/tree/master/sketches) directory. It has versions for both the V2 and V3 designs.

# License

Unless otherwise noted in the source file, this design is copyright and licensed as:

Copyright (c) 2016, Steve Geyer
All rights reserved.

[Creative Commons Attribution 4.0 International](https://creativecommons.org/licenses/by/4.0/)
