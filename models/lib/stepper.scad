smWidth = 42;
smLength = 40;
smScrewHoleOffsets = 31/2;
smCenterDiameter = 22;
smExtra = 1;

module stepperMotor() {
    smCenterHeight = 2;
    smShaftDiameter = 5;
    smShaftLength = 23;
    smScrewDepth = 4.5;
    smScrewDiameter = 3;
    smEdgeTrimOffset = 27;
    module add() {
        translate([-smWidth/2, -smLength, -smWidth/2]) {
            cube([smWidth, smLength, smWidth]);
        }
        rotate([-90, 0, 0]) {
            cylinder(d=smCenterDiameter, h=smCenterHeight, $fn=30);
            cylinder(d=smShaftDiameter, h=smShaftLength, $fn=20);
        }
    }
    module del() {
        for (x = [smScrewHoleOffsets, -smScrewHoleOffsets]) {
            for (z = [smScrewHoleOffsets, -smScrewHoleOffsets]) {
                translate([x, iota, z]) {
                    rotate([90, 0, 0]) {
                        cylinder(d=smScrewDiameter, h=smScrewDepth, $fn=40);
                    }
                }
            }
        }
        for (a = [45, 135, 225, 315]) {
            rotate([0, a, 0]) {
                translate([smEdgeTrimOffset,
                        -smLength-iota, -smWidth/2-iota]) {
                    cube([smWidth, smLength+2*iota, smWidth+2*iota]);
                }
            }
        }

    }
    difference() {
        add();
        del();
    }
}

module stepperMotorHole() {
    smHoleHeight = 20;
    smScrewHoleDiameter = 4;

    widthPlusExtra = smWidth + smExtra;
    lengthPlusExtra = smLength + smExtra/2;

    translate([-smWidth/2, -lengthPlusExtra, -widthPlusExtra/2]) {
        cube([smWidth, lengthPlusExtra, widthPlusExtra]);
    }
    translate([0, -iota, 0]) {
        rotate([-90, 0, 0]) {
            cylinder(d=smCenterDiameter+smExtra, h=smHoleHeight, $fn=40);
        }
    }
    for (x = [smScrewHoleOffsets, -smScrewHoleOffsets]) {
        for (z = [smScrewHoleOffsets, -smScrewHoleOffsets]) {
            translate([x, smHoleHeight-iota, z]) {
                rotate([90, 0, 0]) {
                    cylinder(d=smScrewHoleDiameter, h=smHoleHeight, $fn=40);
                }
            }
        }
    }

}
