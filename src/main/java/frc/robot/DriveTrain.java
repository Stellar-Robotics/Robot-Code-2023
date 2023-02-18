package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveTrain {
    public enum DriveLayout {
        CYGNUS,
        NEW
    }

    public final CANSparkMax driveLeftFront;
    public final CANSparkMax driveRightFront;
    public final CANSparkMax driveLeftRear;
    public final CANSparkMax driveRightRear;

    // Define Controllers

    private final Joystick driverLeft = new Joystick(0);
    private final Joystick driverRight = new Joystick(1);
  
    double driveRight = 0;
    double driveLeft = 0;

    public DriveTrain(DriveLayout layout ) {
        if (layout == DriveLayout.NEW) {
            driveLeftFront = new CANSparkMax(32, MotorType.kBrushless);
            driveRightFront = new CANSparkMax(30, MotorType.kBrushless);
            driveLeftRear = new CANSparkMax(33, MotorType.kBrushless);
            driveRightRear = new CANSparkMax(31, MotorType.kBrushless);
        } else {
            driveLeftFront = new CANSparkMax(50, MotorType.kBrushless);
            driveRightFront = new CANSparkMax(54, MotorType.kBrushless);
            driveLeftRear = new CANSparkMax(42, MotorType.kBrushless);
            driveRightRear = new CANSparkMax(52, MotorType.kBrushless);
        }
    }

    public void tankDrive(boolean toggleState) {

        double driveRight = 0;
        double driveLeft = 0;

        // A small state machine to toggle different drive train speeds.

        double driveSpeedMultiplier = toggleState?0.2:1;

        if (toggleState) {
            if (driverRight.getRawButton(1)) {
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

        // Applying the drive train speed using the raw axis input multiplied by the toggle fraction.
        
        driveLeftFront.set(driveLeft);
        driveRightFront.set(driveRight);
        driveLeftRear.set(driveLeft);
        driveRightRear.set(driveRight);
    }
}
