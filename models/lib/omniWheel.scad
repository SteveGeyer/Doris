omniWheelNumScrews = 4;

// Original motor.
// omniWheelMotorShaftRadius = 3.15;
// omniWheelSlotWidth = 4;
// omniWheelSlotThickness = 2;
// omniWheelSlotIndent = 0.5;
// omniWheelMotorShaftDepth = 11;

// Stepper motor.
omniWheelMotorShaftRadius = 2.6;
omniWheelSlotWidth = 4;
omniWheelSlotThickness = 2;
omniWheelSlotIndent = 0.3;
omniWheelMotorShaftDepth = 15;


omniWheelRadius2 = 30;
omniWheelRadius1 = omniWheelRadius2 - 2.5;

omniWheelLayerHeight = 3;
omniWheelBottomRiserRadius = 9;
omniWheelBottomRiserHeight = 9;
omniWheelBottomRiserRadiusBottom = omniWheelRadius1;
omniWheelBottomNeckHeight = 10;

omniWheelBottomTotalHeight = omniWheelBottomRiserRadius +
                             omniWheelBottomRiserHeight;

omniSlotMargin = 0.45;

omniWheelNumEdgeWheels = 16;
omniEdgeWheelRadius = 5;
omniEdgeWheelHeight = 4.5;

omniEdgeWheelHoleDiameter = 1.5;
omniEdgeWheelChannelWidth = 0.9;

omniEdgeWheelShaftOffset = 0.75;
omniEdgeWheelShaftStub = 2;

omniWheelScrewLengthRoom = 10;
omniWheelScrewHeadDepth = 1;
omniWheelScrewOffset = 13.5;

omniWheelScrewHeaderHeight = omniWheelScrewLengthRoom -
                              2*omniWheelLayerHeight + omniWheelScrewHeadDepth;

omniWheelGrooveDiameter = omniEdgeWheelRadius - 0.5;
omniWheelGrooveHeight = omniEdgeWheelHeight - 1.0;

module edgeWheelWithGroove() {
    $fn=40;
    difference() {
        difference() {
            cylinder(r = omniEdgeWheelRadius, h = omniEdgeWheelHeight);
            translate([0, 0, -iota]) {
                cylinder(d = omniEdgeWheelHoleDiameter,
                         h = omniEdgeWheelHeight+2*iota);
            }
        }
        translate([0, 0, (omniEdgeWheelHeight-omniWheelGrooveHeight)/2]) {
            difference() {
                cylinder(r = omniEdgeWheelRadius+2*iota,
                         h = omniWheelGrooveHeight-2*iota);
                translate([0, 0, -iota]) {
                    cylinder(r = omniWheelGrooveDiameter,
                             h = omniWheelGrooveHeight);
                }
            }
        }
    }
}

module basicWheelAdd() {
    cylinder(r1=omniWheelRadius2, r2=omniWheelRadius1,
        h=omniWheelLayerHeight, $fn=40);
}

module basicWheelDel() {
    $fn=40;
    module edgeWheelSlot() {
        translate([omniWheelRadius1-omniEdgeWheelShaftOffset-
                omniEdgeWheelRadius,
                -omniEdgeWheelHeight/2-omniSlotMargin, -iota]) {
            cube([2*omniEdgeWheelRadius,
                    omniEdgeWheelHeight+2*omniSlotMargin,
                    omniWheelBottomTotalHeight+2*iota
                ]);
        }
        translate([omniWheelRadius1-omniEdgeWheelShaftOffset-
                omniEdgeWheelRadius, 0, -iota]) {
            cylinder(d = omniEdgeWheelHeight+2*omniSlotMargin,
                h = omniWheelBottomTotalHeight+2*iota);
        }
    }

    for (a = [0 : 360/omniWheelNumEdgeWheels : 359]) {
        rotate([0, 0, a]) {
            edgeWheelSlot();
        }
    }
}

module wheelBottom() {
    $fn=40;
    module screwMount() {
        translate([omniWheelScrewOffset, 0, omniWheelLayerHeight]) {
            cylinder(d = screwHeadDiameter, h = omniWheelScrewHeaderHeight);
        }
    }
    module screwHole() {
        translate([omniWheelScrewOffset, 0, -iota]) {
            cylinder(d = screwHoleDiameter,
                h = screwLength-omniWheelLayerHeight+
                    omniWheelScrewHeadDepth);
        }
    }
    module channel() {
        wireLength = omniEdgeWheelHeight+2*omniSlotMargin+
                     2*omniEdgeWheelShaftStub;
        translate([omniWheelRadius1-omniEdgeWheelShaftOffset-
                omniEdgeWheelChannelWidth/2,
                -wireLength/2, -iota]) {
            cube([omniEdgeWheelChannelWidth,
                    wireLength, omniEdgeWheelChannelWidth]);
        }
    }
    module add() {
        basicWheelAdd();
        translate([0, 0, omniWheelLayerHeight]) {
            cylinder(r1=omniWheelBottomRiserRadiusBottom,
                r2=omniWheelBottomRiserRadius,
                h=omniWheelBottomRiserHeight, $fn=80);
        }
        translate([0, 0, omniWheelLayerHeight+omniWheelBottomRiserHeight]) {
            cylinder(r=omniWheelBottomRiserRadius,
                     h=omniWheelBottomNeckHeight, $fn=80);
        }
    }
    module del() {
        translate([0, 0, -iota]) {
            translate([0, 0, omniWheelLayerHeight+omniWheelBottomRiserHeight+
                             omniWheelBottomNeckHeight-
                             omniWheelMotorShaftDepth+2*iota]) {
                difference() {
                    cylinder(r=omniWheelMotorShaftRadius,
                             h=omniWheelMotorShaftDepth); // omniWheelBottomNeckHeight);
                    translate([-omniWheelSlotWidth/2,
                            omniWheelMotorShaftRadius - omniWheelSlotIndent,
                            0]) {
                        cube([omniWheelSlotWidth, omniWheelSlotThickness,
                                omniWheelBottomTotalHeight+2*iota]);
                    }
                }
            }
        }
        for (a = [0 : 360/omniWheelNumScrews : 359]) {
            rotate([0, 0, a]) {
                screwHole();
            }
        }
        for (a = [0 : 360/omniWheelNumEdgeWheels : 359]) {
            rotate([0, 0, a]) {
                channel();
            }
        }
        basicWheelDel();
    }
    difference() {
        add();
        del();
    }
}

module wheelTop() {
    $fn=40;
    module screwHole() {
        translate([omniWheelScrewOffset, 0, -iota]) {
            cylinder(d = screwBodyDiameter,
                h = omniWheelLayerHeight+2*iota);
            translate([0, 0,
                    omniWheelLayerHeight-omniWheelScrewHeadDepth+2*iota]) {
                cylinder(d = screwHeadDiameter,
                    h = omniWheelScrewHeadDepth);
            }
        }
    }

    module add() {
        basicWheelAdd();
    }

    module del() {
        for (a = [0 : 360/omniWheelNumScrews : 359]) {
            rotate([0, 0, a]) {
                screwHole();
            }
        }
        basicWheelDel();
    }

    difference() {
        add();
        del();
    }
}

module omniWheelExplodedView() {
    color("DodgerBlue") {
        wheelBottom();
    }
    color("DeepSkyBlue") {
        translate([0, 0, -20]) {
            rotate([180, 0, 0]) {
                wheelTop();
            }
        }
    }
    rotate([0, 90, 0]) {
        translate([10, omniWheelRadius1, -omniEdgeWheelHeight/2]) {
            edgeWheelWithGroove();
        }
    }
}
