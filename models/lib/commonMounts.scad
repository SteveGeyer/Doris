plateThickness = 3;
middleThickness = 6;
wholeThickness = 2*plateThickness+middleThickness;
mountRadius = 5;
mountBreadth = 15;
rodWithTubing = 5.4;
backplateThickness = 2;
screwOffset = 6.5;         // Hack -- couldn't figure out how to calculate.
threeMotorMountHeight = 16;

module roundedRect(breadth, height, radius) {
    halfb = breadth/2;
    halfr = radius/2;
    linear_extrude(height = height) {
        hull() {
            translate([-halfb+halfr, -halfb+halfr, 0]) {
                circle(r=radius);
            }
            translate([halfb-halfr, -halfb+halfr, 0]) {
                circle(r=radius);
            }
            translate([-halfb+halfr, halfb-halfr, 0]) {
                circle(r=radius);
            }
            translate([halfb-halfr, halfb-halfr, 0]) {
                circle(r=radius);
            }
        }
    }
}

module basicFrameAdd() {
    // basicLength = 39;
    basicLength = 40.5;

    cylinder(r = basicLength, h = plateThickness);
}

module basicFrameDel(cableHole = true) {
    basicCableDiameter = 17;
    basicCutoutOffset = 60;
    basicCutoutDiameter = 45;

    if (cableHole) {
        translate([0, 0, -iota]) {
            cylinder(d = basicCableDiameter,
                h = plateThickness+2*iota, $fn=50);
        }
    }
    for (a = [0, 120, 240]) {
        rotate([0, 0, a]) {
            translate([basicCutoutOffset, 0, -iota]) {
                cylinder(r = basicCutoutDiameter,
                    h = plateThickness+2*iota, $fn=80);
            }
        }
    }
}

module mountFemaleAdd(ledgeHeight = 0) {
    roundedRect(mountBreadth, wholeThickness+ledgeHeight, mountRadius, $fn=30);
}

module mountFemaleDel(ledgeHeight = 0) {
    breadth = mountBreadth + 2*mountRadius;
    if (ledgeHeight > 0) {
        translate([0, 0, -iota+plateThickness]) {
            cylinder(d = rodWithTubing,
                h = wholeThickness+2*iota+ledgeHeight-plateThickness, $fn=40);
        }
    } else {
        translate([0, 0, -iota]) {
            cylinder(d = rodWithTubing,
                h = wholeThickness+2*iota+ledgeHeight, $fn=40);
        }
    }
    for (o = [screwOffset, -screwOffset]) {
        translate([o, iota, wholeThickness/2+ledgeHeight]) {
            rotate([90, 0, 0]) {
                cylinder(d = screwHoleDiameter,
                    h = screwLength-backplateThickness, $fn=10);
            }
        }
    }
    translate([-breadth/2, 0, -iota+ledgeHeight]) {
        cube([breadth, breadth, wholeThickness+2*iota]);
    }
}

module threeMountsAdd(ledgeHeight = 0) {
    for (a = [60, 180, 300]) {
        rotate([0, 0, a]) {
            translate([mountBaseRodOffsets, 0, 0]) {
                rotate([0, 0, -90]) {
                    mountFemaleAdd(ledgeHeight);
                }
            }
        }
    }
}

module threeMountsDel(ledgeHeight = 0) {
    for (a = [60, 180, 300]) {
        rotate([0, 0, a]) {
            translate([mountBaseRodOffsets, 0, 0]) {
                rotate([0, 0, -90]) {
                    mountFemaleDel(ledgeHeight);
                }
            }
        }
    }
}

// ----------------------------------------------------------------------
// Cap for mounts. We need three per mount.
// ----------------------------------------------------------------------
module screwMountCap() {
    breadth = mountBreadth + 2*mountRadius;
    module add() {
        difference() {
            roundedRect(mountBreadth, wholeThickness, mountRadius, $fn=30);
            translate([-breadth/2, 0, -iota]) {
                cube([breadth, breadth, wholeThickness+2*iota]);
            }
        }
    }
    module del() {
        headDiameter = screwHeadDiameter-0.5;
        translate([0, 0, -iota]) {
            cylinder(d = rodWithTubing, h = wholeThickness+2*iota, $fn=40);
        }
        for (o = [screwOffset, -screwOffset]) {
            translate([o, iota, wholeThickness/2]) {
                rotate([90, 0, 0]) {
                    cylinder(d = screwBodyDiameter, h = breadth+2*iota, $fn=40);
                    translate([0, 0, backplateThickness]) {
                        cylinder(d = headDiameter, h = breadth+2*iota, $fn=40);
                    }
                }
            }
        }
    }
    difference() {
        add();
        del();
    }
}

// ----------------------------------------------------------------------
// Mount for the top of the carbon fiber rods.
// ----------------------------------------------------------------------
module topMount() {
    topExtraHeight = plateThickness + 5;
    difference() {
        threeMountsAdd(topExtraHeight);
        threeMountsDel(topExtraHeight);
    }
    difference() {
        basicFrameAdd();
        basicFrameDel();
    }
}

// ----------------------------------------------------------------------
// Interior mount used for support when using one meter rods.
// ----------------------------------------------------------------------
module interiorMount() {
    difference() {
        threeMountsAdd();
        threeMountsDel();
    }
    difference() {
        basicFrameAdd();
        basicFrameDel();
    }
}


// ----------------------------------------------------------------------
// Mount for battery.
// ----------------------------------------------------------------------
batteryRadius = 12.5;
batteryLength = 48;
batteryHolderHeight = 6;
batteryHoleThickness = 5;

module batteryOval(radius, length, height) {
    holeDx = length/2 - radius;
    rectLength = 2*holeDx;
    rectWidth = 2*radius;
    for (o = [-holeDx, holeDx]) {
        translate([o, 0, 0]) {
            cylinder(r = radius, h = height);
        }
    }
    translate([-rectLength/2, -rectWidth/2, 0]) {
        cube([rectLength, rectWidth, height]);
    }
}

module batteryOvalAdd() {
    batteryOval(batteryRadius+plateThickness,
                batteryLength+2*plateThickness,
                batteryHolderHeight+plateThickness);
}

module batteryOvalDel() {
    translate([0, 0, plateThickness+iota]) {
        batteryOval(batteryRadius,
                    batteryLength,
                    batteryHolderHeight);
    }
    translate([0, 0, -iota]) {
        batteryOval(batteryRadius-batteryHoleThickness,
                    batteryLength-2*batteryHoleThickness,
                    plateThickness+batteryHolderHeight+2*iota);
    }
}

module batteryTopMount() {
    difference() {
        union() {
            difference() {
                union() {
                    difference() {
                        basicFrameAdd();
                        basicFrameDel(false);
                    }
                    threeMountsAdd();
                }
                threeMountsDel();
            }
            batteryOvalAdd();
        }
        batteryOvalDel();
    }
}

module batteryBottomMount() {
    mountRadius = 42;
    switchLength = 19.85;
    switchWidth = 13.5;
    switchDx = 32;
    switchDy = -switchLength/2;
    switchBaseRadius = 20;
    switchBaseOffset = 36;
    difference() {
        union() {
            cylinder(r = mountRadius, h = plateThickness, $fn=60);
            translate([switchBaseOffset, 0, 0]) {
                cylinder(r = switchBaseRadius, h = plateThickness, $fn=50);
            }
            batteryOvalAdd();
            threeMountsAdd();
        }
        union() {
            translate([switchDx, switchDy, -iota]) {
                cube([switchWidth, switchLength, plateThickness+2*iota]);
            }
            batteryOvalDel();
            threeMountsDel();
        }
    }
}


// ----------------------------------------------------------------------
// Mount for Uno-style arduino board.
// ----------------------------------------------------------------------
module arduinoMount() {
    arduinoLength = 42;
    arduinoWidth = 10;
    arduinoHeight = 3;
    arduinoCutoutDiameter = 2*arduinoLength-37;
    arduinoRotation = 30;

    arduinoPoleHeight = 15;
    arduinoPoleDiameter = screwHoleDiameter+4;

    arduinoFront = 27.4;
    arduinoFrontLeft = -19.03;
    arduinoFrontRight = 8.93;

    arduinoBack1 = -24.4;
    arduinoBack2 = -23.4;
    arduinoBackLeft = -24.13;
    arduinoBackRight = 24.13;

    offX = 6;
    offY = 0.3;

    subtractYOffset = 39.5;
    subtractEdgesLength = 60;
    subtractEdgesWidth = 9;

    module boardPoleAdd(x, y) {
        translate([x, y, 0]) {
            cylinder(d = arduinoPoleDiameter,
                h = arduinoPoleHeight, $fn=20);
        }
    }

    module boardPoleDel(x, y) {
        translate([x, y, 0]) {
            translate([0, 0, arduinoPoleHeight-screwLength+iota]) {
                cylinder(d = screwHoleDiameter, h = screwLength, $fn=20);
            }
        }
    }

    module arduinoPolesAdd() {
        boardPoleAdd(offX+arduinoFront, offY+arduinoFrontLeft);
        boardPoleAdd(offX+arduinoFront, offY+arduinoFrontRight);
        boardPoleAdd(offX+arduinoBack1, offY+arduinoBackLeft);
        boardPoleAdd(offX+arduinoBack2, offY+arduinoBackRight);
    }

    module arduinoPolesDel() {
        boardPoleDel(offX+arduinoFront, offY+arduinoFrontLeft);
        boardPoleDel(offX+arduinoFront, offY+arduinoFrontRight);
        boardPoleDel(offX+arduinoBack1, offY+arduinoBackLeft);
        boardPoleDel(offX+arduinoBack2, offY+arduinoBackRight);
    }

    module arduinoMountBase() {
        difference() {
            union() {
                difference() {
                    union() {
                        cylinder(r = arduinoLength, h = arduinoHeight, $fn=60);
                    }
                    union() {
                        for (o = [subtractYOffset, -subtractYOffset]) {
                            translate([-subtractEdgesLength/2,
                                    -subtractEdgesWidth/2+o, -iota]) {
                                cube([subtractEdgesLength, subtractEdgesWidth,
                                        arduinoHeight+2*iota]);
                            }
                        }
                        translate([0, 0, -iota]) {
                            cylinder(d = arduinoCutoutDiameter,
                                h = arduinoHeight+2*iota, $fn=60);
                        }
                    }
                }
                arduinoPolesAdd();
            }
            arduinoPolesDel();
        }
    }

    difference() {
        union() {
            arduinoMountBase();
            threeMountsAdd();
        }
        threeMountsDel();
    }
}


// ----------------------------------------------------------------------
// Two motor mount for segway-style base.
// ----------------------------------------------------------------------
centerDiameter = 24;
mountScrewDiameter = 4;
mountScrewOffset = 5.5;
ledgeHeight = 2*mountScrewOffset;

holeDiameter = 15;

baseLength = 110;
baseWidth = smWidth+2*plateThickness;


module twoWheelEndAssembly() {
    module sideBrace() {
        rotate([90, 0, 0]) {
            difference() {
                cube([ledgeHeight+plateThickness, ledgeHeight, plateThickness]);
                translate([plateThickness, ledgeHeight+iota, -iota]) {
                    rotate([0, 0, -45]) {
                        cube([2*ledgeHeight, 2*ledgeHeight,
                                plateThickness+2*iota]);
                    }
                }
            }
        }
    }
    module endPiece() {
        edgeOffset = smWidth/2 - mountScrewOffset;
        smWidthWithIotas = smWidth+2*iota;

        translate([-smWidth/2, 0, 0]) {
            difference() {
                difference() {
                    translate([-smWidth/2, -smWidth/2, 0]) {
                        cube([smWidth, smWidth, plateThickness]);
                    }
                    union() {
                        translate([0, 0, -iota]) {
                            cylinder(d=centerDiameter,
                                h=plateThickness+2*iota, $fn=40);
                        }
                        for (dy = [edgeOffset, -edgeOffset]) {
                            translate([edgeOffset, dy, -iota]) {
                                cylinder(d=mountScrewDiameter,
                                    h=plateThickness+2*iota, $fn=20);
                            }
                        }
                    }
                }
                translate([-smWidthWithIotas/2-ledgeHeight,
                        -smWidthWithIotas/2, -iota]) {
                    cube([smWidthWithIotas, smWidthWithIotas,
                            plateThickness+2*iota]);
                }
            }
        }
    }
    translate([0, 0, plateThickness]) {
        rotate([0, 90, 0]) {
            endPiece();
        }
    }
    for (y = [-baseWidth/2+plateThickness, baseWidth/2]) {
        translate([0, y, plateThickness-iota]) {
            sideBrace();
        }
    }
}

module twoWheelBaseCore() {
    baseWidth = smWidth+2*plateThickness;
    translate([0, -baseWidth/2, 0]) {
        cube([baseLength, baseWidth, plateThickness]);
    }
    translate([0, 0, -iota]) {
        twoWheelEndAssembly();
        translate([baseLength, 0, 0]) {
            rotate([0, 0, 180]) {
                twoWheelEndAssembly();
            }
        }
    }
}

module twoWheelBaseTop() {
    interiorHoleDiameter = 25;
    holeDx = 17;
    holeDy = 15;
    screwHeadHeight = 1.5;
    module screwHolesDel() {
        for (dx = [holeDx, -holeDx]) {
            for (dy = [holeDy, -holeDy]) {
                translate([baseLength/2+dx, dy, -iota]) {
                    cylinder(d=screwBodyDiameter,
                        h=plateThickness+2*iota, $fn = 20);
                    translate([0, 0, plateThickness-screwHeadHeight+2*iota]) {
                        cylinder(d=screwHeadDiameter,
                            h=screwHeadHeight, $fn = 10);
                    }

                }
            }
        }
    }
    translate([-baseLength/2, 0, 0]) {
        difference() {
            twoWheelBaseCore();
            union() {
                translate([baseLength/2, 0, -iota]) {
                    scale([2, 1, 1]) {
                        cylinder(d=interiorHoleDiameter,
                            h=plateThickness+2*iota, $fn=40);
                    }
                }
                screwHolesDel();
            }
        }
    }
}

module twoWheelBaseBottom() {
    translate([-baseLength/2, 0, 0]) {
        twoWheelBaseCore();
    }
}

module twoWheelMotorMount() {
    mountRadius = 42;
    extensionHeight = 0;
    interiorHoleDiameter = 25;
    holeDx = 15;
    holeDy = 17;
    screwTopDiameter = 6;
    screwTopHeight = screwLength - plateThickness + 2;
    module screwAdd() {
        for (dx = [holeDx, -holeDx]) {
            for (dy = [holeDy, -holeDy]) {
                translate([dx, dy, 0]) {
                    cylinder(d=screwTopDiameter, h=screwTopHeight, $fn = 20);
                }
            }
        }
    }
    module screwDel() {
        for (dx = [holeDx, -holeDx]) {
            for (dy = [holeDy, -holeDy]) {
                translate([dx, dy, -iota]) {
                    cylinder(d=screwHoleDiameter,
                        h=screwLength-plateThickness+iota, $fn = 10);
                }
            }
        }
    }

    difference() {
        union() {
            cylinder(r = mountRadius, h = plateThickness, $fn=60);
            threeMountsAdd(plateThickness);
            screwAdd();
        }
        union() {
            threeMountsDel(plateThickness);
            translate([0, 0, -iota]) {
                scale([1, 2, 1]) {
                    cylinder(d=interiorHoleDiameter,
                        h=plateThickness+2*iota, $fn = 40);
                }
            }
            screwDel();
        }
    }
}


// ----------------------------------------------------------------------
// Three motor mount for ballbot base.
// ----------------------------------------------------------------------
threeWheelBottomConeDiameter = 150;
threeWheelBottomHoleDiameter = 105;
threeWheelBottomHeight = 45;

module cone(bottomDiameter, height) {
    cylinder(d1 = bottomDiameter,
        d2 = bottomDiameter-2*height,
        h = height, $fn=200);
}

module threeMotorMount() {
    extensionHeight = 17;
    slide = 3;
    wireHoleDiameter = 12;
    wireHoleHeight = 100;
    difference() {
        union() {
            cone(threeWheelBottomConeDiameter, threeWheelBottomHeight);
            translate([0, 0, threeMotorMountHeight]) {
                threeMountsAdd(threeMotorMountHeight);
            }
        }
        union() {
            translate([0, 0, 18]) {
                for (a = [60, 180, 300]) {
                    rotate([0, 0, a]) {
                        translate([-70, 0, 0]) {
                            rotate([-90, 45, 0]) {
                                stepperMotorHole();
                            }
                        }
                    }
                }
            }
            translate([0, 0, -iota]) {
                cylinder(d = 40, h = 45+2*iota, $fn=100);
                cone(105, 45+2*iota);
            }
            for (a = [60, 180, 300]) {
                rotate([0, 45, a-60]) {
                    translate([2, 0, 0]) {
                        scale([0.9, 1, 1]) {
                            cylinder(d = wireHoleDiameter, h = wireHoleHeight, $fn = 40);
                        }
                    }
                }
                rotate([0, 0, a]) {
                    translate([-55, 0, 0]) {
                        rotate([0, 45, 0]) {
                            translate([-100, -100, -200]) {
                                cube([200, 200, 200]);
                            }
                        }
                    }
                }
            }
            translate([0, 0, threeMotorMountHeight]) {
                threeMountsDel(threeMotorMountHeight);
            }
        }
    }
}

threeMotorStandVerticalHeight = 35;
threeMotorStandConeHeight = 17.5;
threeMotorStandBrim = 10;

module threeMotorStand() {
    holeDiameter = 12;
    holeLength = threeWheelBottomHoleDiameter+2*iota;
    standThickness = 2;
    difference() {
        union() {
            translate([0, 0, threeMotorStandVerticalHeight]) {
                cone(threeWheelBottomHoleDiameter, threeMotorStandConeHeight);
            }
            cylinder(d = threeWheelBottomHoleDiameter,
                h = threeMotorStandVerticalHeight, $fn=200);

            cylinder(d = threeWheelBottomHoleDiameter,
                h = threeMotorStandVerticalHeight, $fn=200);
            cylinder(d = threeWheelBottomHoleDiameter+2*threeMotorStandBrim,
                h = plateThickness, $fn=200);
        }
        union() {
            translate([0, 0, threeMotorStandVerticalHeight]) {
                cone(threeWheelBottomHoleDiameter-2*standThickness,
                    threeMotorStandConeHeight+iota);
            }
            translate([0, 0, plateThickness+iota]) {
                cylinder(d = threeWheelBottomHoleDiameter-2*standThickness,
                    h = threeMotorStandVerticalHeight-plateThickness+iota, $fn=200);
            }
            translate([0, 0, -iota]) {
                cylinder(d = threeWheelBottomHoleDiameter-2*threeMotorStandBrim,
                    h = plateThickness+3*iota, $fn=200);
            }

            for (a = [0, 20, 40, 60, 80, 100, 120, 140, 160]) {
                rotate([0, 0, a]) {
                    translate([0, 0, threeMotorStandVerticalHeight+threeMotorStandConeHeight/2]) {
                        rotate([90, 0, 0]) {
                            translate([0, 0, -holeLength/2]) {
                                cylinder(d = holeDiameter, h = holeLength, $fn=50);
                            }
                        }
                    }
                    translate([0, 0, threeMotorStandVerticalHeight/2+plateThickness/2]) {
                        rotate([90, 0, 0]) {
                            translate([0, 0, -holeLength/2]) {
                                scale([1, 2, 1]) {
                                    cylinder(d = holeDiameter, h = holeLength, $fn=50);
                                }
                            }
                        }
                    }

                }
            }
        }
    }
}


// ----------------------------------------------------------------------
// Show all the parts in an exploded parts manner.
// ----------------------------------------------------------------------

module commonBreakdown() {
    cfHeight = 333;
    cfDiameter = 5;

    // Start with the Rods.
    color("darkGrey") {
        for (a = [60, 180, 300]) {
            rotate([0, 0, a]) {
                translate([mountBaseRodOffsets, 0, 0]) {
                    cylinder(d = cfDiameter, h = cfHeight, $fn=30);
                }
            }
        }
    }

    // Start with base.
    color("deepSkyBlue") {
        translate([0, 0, cfHeight]) {
            rotate([180, 0, 0]) {
                topMount();
            }
        }
    }

    color("lightBlue") {
        translate([0, 0, cfHeight-50]) {
            interiorMount();
        }
    }

    color("lightCoral") {
        translate([0, 0, cfHeight-100]) {
            rotate([180, 0, 0]) {
                batteryTopMount();
            }
        }
    }

    color("lightCoral") {
        translate([0, 0, cfHeight-233]) {
            batteryBottomMount();
        }
    }

    color("springGreen") {
        translate([0, 0, 40]) {
            arduinoMount();
        }
    }
}

// Show all the ballbot parts.
module ballBotExplodedView() {
    commonBreakdown();
    color("deepSkyBlue") {
        translate([0, 0, -(threeMotorMountHeight+plateThickness)]) {
            threeMotorMount();
        }
    }
}

// Show all the ballbot parts.
module twoWheelExplodedView() {
    commonBreakdown();
    color("deepSkyBlue") {
        translate([0, 0, -plateThickness]) {
            twoWheelMotorMount();
        }
    }
    color("firebrick") {
        translate([0, 0, -20]) {
            rotate([180, 0, 90]) {
                twoWheelBaseTop();
            }
        }
        translate([0, 0, -62]) {
            rotate([0, 0, 90]) {
                twoWheelBaseBottom();
            }
        }
    }
}
