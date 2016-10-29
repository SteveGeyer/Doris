// A small quantity to keep parts from starting or ending at exactly the
// same location. For example when we create a hole through an object we
// want the object being subtracted to stick slightly past the object being
// subtracted from.
iota = 0.1;


// Length and hole diameter of the #4 x 3/8in Phillips mount screw.
screwLength = 9.3;
screwHoleDiameter = 2.75;
screwHeadDiameter = 6.5;
screwBodyDiameter = 3.4;


// These constants are the basic parameters of the mount plate. They are
// necessary for other components to make their calculations.
mountPlateWidth = 37.6;
mountPlateLength = 51;


// These constants are the basis for the carbon fiber mount points. They are
// necessary for other components to make their calculations.
mountRodDiameter = 5.5;         // A little extra because holes are smaller.
mountThickness = 9;
mountDiameter = mountRodDiameter + 2*mountThickness;


// These constasnts are teh basis for the motor base. They are necessary for
// other components to make their calculations.
motorBaseExtraAxisOffset = 5;
motorBaseTopOffset = sqrt(3)*mountPlateWidth/6 + motorBaseExtraAxisOffset;
motorBaseBottomOffset = mountPlateLength/sqrt(2) + motorBaseTopOffset;
motorBaseBottomRadius = sqrt(pow(motorBaseBottomOffset,2) +
                             pow(mountPlateWidth/2, 2));
mountBaseRodOffsets = motorBaseBottomRadius-mountDiameter/2;
