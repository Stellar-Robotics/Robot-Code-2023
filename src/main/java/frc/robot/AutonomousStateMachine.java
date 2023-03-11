package frc.robot;

import java.util.Random;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousStateMachine {
    private Robot robot;

    // AUTO STUFF
    enum State {
        DRIVE_TO_PLATFORM,
        BALANCE
    }

    private State currentState = State.DRIVE_TO_PLATFORM;

    // PID stuff
    double pitchIAccumulator = 0;
    double yawP = 0.005;
    double pitchIReset = 4;

    public AutonomousStateMachine(Robot robot, State startingState) {
        this.robot = robot;
        this.currentState = startingState;
    }

    public AutonomousStateMachine(Robot robot) {
        this(robot, State.DRIVE_TO_PLATFORM);
    }

    public void run() {
        SmartDashboard.putString("CURRENT STATE", currentState.toString());
        
        switch(currentState) {
            case DRIVE_TO_PLATFORM: {this.driveToPlatform(); break;}
            case BALANCE: {this.balance2(); break;}
        }
    }

    public void driveToPlatform() {
        SmartDashboard.putNumber("STATE MACHINE RANDOM", new Random().nextDouble());

        robot.DRIVE_LEFT.setReference(-0.5, ControlType.kDutyCycle);
        robot.DRIVE_RIGHT.setReference(0.5, ControlType.kDutyCycle);

        if (Math.abs(robot.gyro.getYComplementaryAngle()) > 10) {
            currentState = State.BALANCE;
        }
    }

    public void balance() {
        double roll = robot.gyro.getXComplementaryAngle();
        double yaw = robot.gyro.getAngle();
        double pitch = -robot.gyro.getYComplementaryAngle();
        double pitchVel = -robot.gyro.getRate();
        
        SmartDashboard.putNumber("Yaw", yaw);
    
        SmartDashboard.putNumber("Pitch", pitch);
        SmartDashboard.putNumber("Pitch Velocity", pitchVel);
        SmartDashboard.putNumber("roll", roll);
    
        pitchIAccumulator = Math.abs(pitch) > pitchIReset? pitchIAccumulator + pitch: 0;
    
        SmartDashboard.putNumber("Accumulated I", pitchIAccumulator);
    
    
        double pitchD = SmartDashboard.getNumber("D", 0);
        double pitchP = SmartDashboard.getNumber("P", 0);
        double pitchI = SmartDashboard.getNumber("I", 0);

        double pitchExponent = SmartDashboard.getNumber("PitchExponent", 1);
    
        //double stopAngle = SmartDashboard.getNumber("StopAngle", 4);
        
        // What is the maximum angular velocity at which we apply the D term?
        double pitchMaxDVelocity = SmartDashboard.getNumber("PitchMaxDVelocity", 100);
        
        // Calculate PID gains for pitch control
        double pitchForceP = (pitchP * Math.pow(Math.abs(pitch), pitchExponent) * Math.signum(pitch));
        double pitchForceI = (pitchI * pitchIAccumulator);
        double pitchForceD = (pitchD * (Math.abs(pitchVel) < pitchMaxDVelocity? pitchVel : 0));
        
        
        // Calculate total power for correcting pitch
        double pitchForce = pitchForceP + pitchForceI + pitchForceD;
        
        // Calculate PID gains for yaw control and add to pitch power
        double driveLeftPower =  -pitchForce + (yawP * yaw);
        double driveRightPower = pitchForce + (yawP * yaw);
        
        SmartDashboard.putNumber("power", pitchForce);
        
        // Apply power
        //robot.drivetrain.tankDrive(driveLeftPower, driveRightPower);
        robot.DRIVE_LEFT.setReference(driveLeftPower, ControlType.kDutyCycle);
        robot.DRIVE_RIGHT.setReference(driveRightPower, ControlType.kDutyCycle);

    }

    public void balance2() {
        double roll = robot.gyro.getXComplementaryAngle();
        double yaw = robot.gyro.getAngle();
        double pitch = -robot.gyro.getYComplementaryAngle();
        double pitchVel = -robot.gyro.getRate();

        SmartDashboard.putNumber("Pitch", pitch);
        SmartDashboard.putNumber("Pitch Velocity", pitchVel);
        SmartDashboard.putNumber("Accumulated I", pitchIAccumulator);

        double pitchP = SmartDashboard.getNumber("P", 0);
        double pitchI = SmartDashboard.getNumber("I", 0);
        double pitchD = SmartDashboard.getNumber("D", 0);

        double pitchForce = 0;

        if (Math.abs(pitchVel) < 5) {
            SmartDashboard.putNumber("ENCODER", robot.DRIVE_LEFT_FRONT.getEncoder().getVelocity());
            
            if (Math.abs(robot.DRIVE_LEFT_FRONT.getEncoder().getVelocity()) < 100 && Math.abs(pitch) > 5) {
                pitchIAccumulator += pitchI * Math.signum(pitch);
            } else {
                pitchIAccumulator = 0;
            }

            //pitchForce = Math.pow(Math.abs(pitch), pitchExponent) * Math.signum(pitch) * pitchP;
            pitchForce = (pitch + pitchIAccumulator) * pitchP;
        } else {
            pitchForce = pitchVel * pitchD;
        }

        // Calculate PID gains for yaw control and add to pitch power
        double driveLeftPower =  -pitchForce + (yawP * yaw);
        double driveRightPower = pitchForce + (yawP * yaw);
        
        SmartDashboard.putNumber("power", pitchForce);
        
        // Apply power
        robot.DRIVE_LEFT.setReference(driveLeftPower, ControlType.kDutyCycle);
        robot.DRIVE_RIGHT.setReference(driveRightPower, ControlType.kDutyCycle);
    }
}
