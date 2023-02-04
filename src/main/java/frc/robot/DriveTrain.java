package frc.robot;

public class DriveTrain {

    /*public void tankDrive() {

        double driveRight = 0;
        double driveLeft = 0;
      
        boolean toggleState = false;
        boolean toggleLast = false;

                // A small state machine to toggle different drive train speeds.
        double driveSpeedMultiplier = toggleState?0.2:1;
        if (toggleState) {
        if (Robot.driverRight.getRawButton(1)) {
            driveLeft = driverRight.getZ() * driveSpeedMultiplier * 0.8;
            driveRight = driverRight.getZ() * driveSpeedMultiplier * 0.8;
        }
        else{
            driveLeft = -driverLeft.getY() * driveSpeedMultiplier;
            driveRight = driverRight.getY() * driveSpeedMultiplier;
        }
        }
        else {
        if (driverRight.getRawButton(1)) {
            driveLeft = driverRight.getZ() * driveSpeedMultiplier * 0.6;
            driveRight = driverRight.getZ() * driveSpeedMultiplier * 0.6;
        }
        else{
            driveLeft = -driverLeft.getY() * driveSpeedMultiplier;
            driveRight = driverRight.getY() * driveSpeedMultiplier;
        }
        }
        if ( (driverLeft.getRawButton(12) == true) & (!toggleLast) ) {
        toggleState  = !toggleState;
        }

        toggleLast = driverLeft.getRawButton(12);


        // Applying the drive train speed using the raw axis input multiplied by the toggle fraction.
        driveLeftFront.set(driveLeft);
        driveRightFront.set(driveRight);
        driveLeftRear.set(driveLeft);
        driveRightRear.set(driveRight);
    }*/
}
