package frc.robot;

import java.util.Random;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousStateMachine {
    class BoxcarAverager {
        double[] values;
        int current = 0;

        public BoxcarAverager(int size) {
            values = new double[size];
        }

        public void addValue(double value) {
            values[current] = value;
            if (++current == values.length) {current = 0;}
        }

        public double getValue() {
            double sum = 0;
            for (double val : values) {
                sum += val;
            }
            return sum / values.length;
        }
    }

    private Robot robot;

    // AUTO STUFF
    enum State {
        DRIVE_TO_PLATFORM,
        BALANCE,
        DO_NOTHING,
        DRIVE_TO_LINE
    }

    private State currentState;

    // PID stuff
    double pitchIAccumulator = 0;
    double yawP = 0.005;
    double pitchIReset = 4;

    double previousPitch = 0;
    long previousTime = 0;

    BoxcarAverager pitchVelocityAverager;

    public AutonomousStateMachine(Robot robot, State startingState) {
        this.robot = robot;
        this.currentState = startingState;
        pitchVelocityAverager = new BoxcarAverager(100);
    }

    public AutonomousStateMachine(Robot robot) {
        this(robot, State.DRIVE_TO_PLATFORM);
    }

    public void run() {
        SmartDashboard.putString("CURRENT STATE", currentState.toString());
        
        switch(currentState) {
            case DRIVE_TO_PLATFORM: {this.driveToPlatform(); break;}
            case DRIVE_TO_LINE: {this.driveToLine(); break;}
            case BALANCE: {this.balance2(); break;}
            default: {break;}
        }
    }

    public void driveToPlatform() {
        //SmartDashboard.putNumber("STATE MACHINE RANDOM", new Random().nextDouble());
        double yaw = robot.gyro.getAngle();
        double pitch = -robot.gyro.getYComplementaryAngle();
        //double pitchVel = -robot.gyro.getRate();
        pitchVelocityAverager.addValue((pitch - previousPitch) / (System.currentTimeMillis() - previousTime) * 1000);
        double pitchVel = pitchVelocityAverager.getValue();
        previousTime = System.currentTimeMillis();
        previousPitch = pitch;

        SmartDashboard.putNumber("Pitch", pitch);
        SmartDashboard.putNumber("Pitch Velocity", pitchVel);

        // Drive forward and steer
        robot.DRIVE_LEFT.setReference(-0.15 + (yawP * yaw), ControlType.kDutyCycle);
        robot.DRIVE_RIGHT.setReference(0.15 + (yawP * yaw), ControlType.kDutyCycle);

        if (pitchVel < -4) {
            currentState = State.BALANCE;
        }
    }

    public void driveToLine() {
        final double LINE_ENCODER_COUNT = 3000;

        //SmartDashboard.putNumber("STATE MACHINE RANDOM", new Random().nextDouble());
        double yaw = robot.gyro.getAngle();

        // Drive forward and steer
        robot.DRIVE_LEFT.setReference(-0.15 + (yawP * yaw), ControlType.kDutyCycle);
        robot.DRIVE_RIGHT.setReference(0.15 + (yawP * yaw), ControlType.kDutyCycle);
        
        if (robot.DRIVE_LEFT_FRONT.getEncoder().getPosition() >= LINE_ENCODER_COUNT) {
            robot.DRIVE_LEFT.setReference(0, ControlType.kDutyCycle);
            robot.DRIVE_RIGHT.setReference(0, ControlType.kDutyCycle);
            currentState = State.DO_NOTHING;
        }
    }

    public void balance() {
        double roll = robot.gyro.getXComplementaryAngle();
        double yaw = robot.gyro.getAngle();
        double pitch = -robot.gyro.getYComplementaryAngle();
        //double pitchVel = -robot.gyro.getRate();
        pitchVelocityAverager.addValue((pitch - previousPitch) / (System.currentTimeMillis() - previousTime) * 1000);
        double pitchVel = pitchVelocityAverager.getValue();
        previousTime = System.currentTimeMillis();
        previousPitch = pitch;

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

        if (Math.abs(pitchVel) < 4) {
            SmartDashboard.putNumber("ENCODER", robot.DRIVE_LEFT_FRONT.getEncoder().getVelocity());
            double velocityTerm = 0;

            if (Math.abs(pitch) > 5) {
                velocityTerm = (80 - Math.abs(robot.DRIVE_LEFT_FRONT.getEncoder().getVelocity())) * Math.signum(pitch) * pitchI;
                //pitchIAccumulator += pitchI * Math.signum(pitch);
            } else {
                pitchIAccumulator = 0;
            }

            //pitchForce = Math.pow(Math.abs(pitch), pitchExponent) * Math.signum(pitch) * pitchP;
            pitchForce = (pitch + pitchIAccumulator + velocityTerm) * pitchP;
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
