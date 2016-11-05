// ----------------------------------------------------------------------
// Omniwheel connector.
// ----------------------------------------------------------------------

module omniWheelConnector() {
    omniWheelMotorShaftRadius = 2.6;
    omniWheelSlotWidth = 4;
    omniWheelSlotThickness = 2;
    omniWheelSlotIndent = 0.3;
    omniWheelMotorShaftDepth = 15;

    boltHeight = 11;
    boltConeHeight = 1;
    boltWidth = 11.2;
    shaftWidth = 6.3;
    collarWidth = 18;
    collarHeight = 1;

    slotWidth = 4;
    slotIndent = 0.5;              // Was 0.6

    shaftHeight = boltHeight + collarHeight;

    exhaustHoleWidth = 1.5;
    longLength = 100;
    shaftLength = 30;
    capHeight = shaftLength - boltHeight;

    capExtraHeight = 3;
    wireHoleWidth = 2;

    module hexagon(size, height) {
        boxWidth = size/1.75;
        for (r = [-60, 0, 60]) {
            rotate([0, 0, r]) {
                translate([0, 0, height/2]) {
                    cube([boxWidth, size, height], true);
                }
            }
        }
    }
    intersection() {
        difference() {
            union() {
                difference() {
                    union() {
                        cylinder(r = collarWidth/2, h = collarHeight);
                        translate([0, 0, 1]) {
                            hexagon(boltWidth, boltHeight+capHeight+capExtraHeight);
                        }
                    }
                    union() {
                        difference() {
                            cylinder(r=omniWheelMotorShaftRadius,
                                h=omniWheelMotorShaftDepth, $fn=40);
                            translate([-omniWheelSlotWidth/2,
                                    omniWheelMotorShaftRadius - omniWheelSlotIndent,
                                    0]) {
                                cube([omniWheelSlotWidth, omniWheelSlotThickness,
                                        omniWheelMotorShaftDepth+2*iota]);
                            }
                        }
                        cylinder(r = exhaustHoleWidth/2, h = longLength, $fn=20);
                        translate([2.5, longLength/2,
                                collarHeight+boltHeight+
                                capHeight+wireHoleWidth/2]) {
                            rotate([90, 0, 0]) {
                                cylinder(r = wireHoleWidth/2,
                                    h = longLength, $fn=20);
                            }
                        }
                    }
                }
                translate([-slotWidth/2, shaftWidth/2 - slotIndent, 0]) {
                    cube([slotWidth, 1, shaftHeight]);
                }
            }
            translate([0, 0, -iota]) {
                cylinder(r1 = omniWheelMotorShaftRadius+0.5, r2 = omniWheelMotorShaftRadius,
                    h = boltConeHeight, $fn=40);
            }
        }
        sphere(collarHeight + boltHeight + capHeight + capExtraHeight);
    }
}
