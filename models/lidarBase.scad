include <lib/commonConstants.scad>;
include <lib/commonMounts.scad>;
include <lib/lidar.scad>;

lidarServoHolder();
translate([-32.5/2, -18, 0]) {
    cube([32.5, 36, plateThickness]);
}
