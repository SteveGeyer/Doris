module lidarServoHolder() {
    length = 23.5;
    breadth = 12.7;
    height = 16;
    tabThickness = 4.5;
    screwHoleDistance = 28;
    screwDiameter = 2.4;
    screwHeight = 10;
    wallThickness = 1;
    totalLength = length+2*tabThickness;
    totalBreath = breadth+2*wallThickness;
    connectorWidth = 4;

    translate([-totalLength/2, 0, 0]) {
        difference() {
            translate([0, -totalBreath/2, 0]) {
                cube([totalLength, totalBreath, height+2*plateThickness]);
            }
            union() {
                translate([tabThickness, -totalBreath/2+wallThickness,
                           plateThickness+iota]) {
                    cube([length, breadth, height+plateThickness]);
                }
                for (dx = [-screwHoleDistance/2, screwHoleDistance/2]) {
                    translate([totalLength/2+dx, 0,
                               height+2*plateThickness-screwHeight+iota]) {
                        cylinder(d = screwDiameter, h = screwHeight, $fn=20);
                    }
                }
                translate([-iota, 0, plateThickness+iota]) {
                    scale([1, 1.5, 1]) {
                        difference() {
                            rotate([0, 90, 0]) {
                                cylinder(d = 2*connectorWidth,
                                         h = tabThickness+2*iota, $fn=40);
                            }
                            translate([-iota, -connectorWidth, -connectorWidth]) {
                                cube([tabThickness+4*iota,
                                      2*connectorWidth, connectorWidth]);
                            }
                        }
                    }
                }
            }
        }
    }
}

hornDiameter = 8;
hornHeight = 4;
hornHoleDepth = 2.5;
hornHoleSquareWidth = 3.5;
hornScrewDiameter = 3;
hornScrewHeadDiameter = 6.5;
headMountScrewOffset = 6.4;
connectorWidth = 18;
connectorHeight = 2.6;
screwHeadThickness = 1.5;
headLength = connectorWidth + 2*hornDiameter;
headHeight = screwLength - screwHeadThickness + 0.5;
headEdgeOffset = (headLength - 2*headMountScrewOffset)/2;
headMountScrewDiameter = 2;
headHoleOffset = headLength/2 - hornDiameter/2;


// screwLength = 9.3;
// screwHoleDiameter = 2.75;
// screwBodyDiameter = 3.4;


module servoAdd() {
    cylinder(d=hornDiameter, h=hornHeight, $fn=35);
}

module servoDel() {
    union() {
        translate([0, 0, -iota]) {
            cylinder(d=hornScrewDiameter, h=hornHeight+2*iota, $fn=20);
        }
        for (r = [0 : 90.0/5 : 90]) {
            rotate([0, 0, r]) {
                translate([-hornHoleSquareWidth/2, -hornHoleSquareWidth/2,
                        hornHeight-hornHoleDepth+iota]) {
                    cube([hornHoleSquareWidth, hornHoleSquareWidth, hornHoleDepth]);
                }
            }
        }
    }
}

module lidarHeadPart1() {
    translate([0, 0, headHeight]) {
        difference() {
            servoAdd();
            servoDel();
        }
    }
    difference() {
        translate([-hornDiameter/2, -headLength/2, 0]) {
            cube([hornDiameter, headLength, headHeight]);
        }
        union() {
            translate([0, 0, -iota]) {
                cylinder(d = hornScrewHeadDiameter,
                    h = headHeight+2*iota, $fn = 30);
            }
            for (o = [headHoleOffset, -headHoleOffset]) {
                translate([0, o, -iota]) {
                    cylinder(d = screwHoleDiameter,
                        h = screwLength - screwHeadThickness, $fn=20);
                }
            }
        }
    }
}

module lidarHeadPart2() {
    difference() {
        translate([-hornDiameter/2, -headLength/2, 0]) {
            cube([hornDiameter, headLength, 2*connectorHeight]);
        }
        union() {
            translate([-hornDiameter/2-iota,
                      -connectorWidth/2, connectorHeight+iota]) {
                cube([hornDiameter+2*iota, connectorWidth, connectorHeight]);
            }
            for (o = [headHoleOffset, -headHoleOffset]) {
                translate([0, o, -iota]) {
                    cylinder(d = screwBodyDiameter,
                             h = 2*connectorHeight+2*iota, $fn=20);
                    cylinder(d = screwHeadDiameter,
                             h = 2*connectorHeight-screwHeadThickness, $fn=20);
                }
            }
        }
    }
}
