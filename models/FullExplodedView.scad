include <lib/commonConstants.scad>;
include <lib/stepper.scad>;
include <lib/commonMounts.scad>;
include <lib/omniWheel.scad>;

translate([40, -150, 310]) {
    rotate([0, 0, 30]) {
        screwMountCap();
    }
}

translate([0, -95, 0]) {
    twoWheelExplodedView();
}
translate([0, 95, 0]) {
    ballBotExplodedView();
}
translate([0, 200, -40]) {
    rotate([45, 0, 0]) {
        omniWheelExplodedView();
    }
}
