package frc.robot;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;

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
        DRIVE_TO_LINE,
        DRIVE_TO_WALL,
        SCORE_CONE,
        RAISE_ARM,
        LOWER_ARM
    }

    // The minimum angle at which the robot stops driving forward at
    // a constant speed and begins driving more slowly to balance on the platform.
    final double START_BALANCING_AT_ANGLE = 18;

    private State currentState;

    // PID stuff
    double pitchIAccumulator = 0;
    double yawP = 4;
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
            case DO_NOTHING: {break;}
            case DRIVE_TO_PLATFORM: {this.driveToPlatform(); break;}
            case DRIVE_TO_LINE: {this.driveToLine(); break;}
            case BALANCE: {this.balance2(); break;}
            case RAISE_ARM: {this.raiseArm(); break;}
            case DRIVE_TO_WALL: {this.driveToWall(); break;}
            case SCORE_CONE: {this.scoreCone(); break;}
            case LOWER_ARM: {break;}
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
        robot.DRIVE_LEFT.setReference(600 + (yawP * yaw), ControlType.kVelocity);
        robot.DRIVE_RIGHT.setReference(-600 + (yawP * yaw), ControlType.kVelocity);

        if (Math.abs(pitch) > START_BALANCING_AT_ANGLE) {
            currentState = State.BALANCE;
        }
    }

    public void driveToLine() {
        final double LINE_ENCODER_COUNT = 30;

        //SmartDashboard.putNumber("STATE MACHINE RANDOM", new Random().nextDouble());
        //double yaw = robot.gyro.getAngle();

        // Drive forward and steer
        robot.DRIVE_LEFT.setReference(-0.15, ControlType.kDutyCycle);
        robot.DRIVE_RIGHT.setReference(0.15, ControlType.kDutyCycle);

        SmartDashboard.putNumber("drivetrain encoder distance", robot.DRIVE_LEFT_FRONT.getEncoder().getPosition());
        
        if (robot.DRIVE_LEFT_FRONT.getEncoder().getPosition() <= -LINE_ENCODER_COUNT) {
            robot.DRIVE_LEFT.setReference(0, ControlType.kDutyCycle);
            robot.DRIVE_RIGHT.setReference(0, ControlType.kDutyCycle);
            currentState = State.DO_NOTHING;
        }
    }

    public void balance2() {
        //double roll = robot.gyro.getXComplementaryAngle();
        double yaw = robot.gyro.getAngle();
        double pitch = -robot.gyro.getYComplementaryAngle();
        double pitchVel = -robot.gyro.getRate();

        SmartDashboard.putNumber("Pitch", pitch);
        SmartDashboard.putNumber("Pitch Velocity", pitchVel);
        SmartDashboard.putNumber("Accumulated I", pitchIAccumulator);

        double pitchP = SmartDashboard.getNumber("PitchP", 22);
        //double pitchI = SmartDashboard.getNumber("PitchI", 0);
        double pitchD = SmartDashboard.getNumber("PitchD", 10);

        SmartDashboard.putNumber("PitchVelThresh", 4.5);

        double pitchForce = 0;

        if (Math.abs(pitchVel) < SmartDashboard.getNumber("PitchVelThresh", 4.5)) {
            pitchForce = pitch * pitchP;
        } else {
            pitchForce = pitchVel * pitchD;
        }

        // Calculate PID gains for yaw control and add to pitch power
        double driveLeftPower =  -pitchForce + (yawP * yaw);
        double driveRightPower = pitchForce + (yawP * yaw);
        
        SmartDashboard.putNumber("power", pitchForce);
        
        // Apply power
        robot.DRIVE_LEFT.setReference(driveLeftPower, ControlType.kVelocity);
        robot.DRIVE_RIGHT.setReference(driveRightPower, ControlType.kVelocity);
    }

    public void raiseArm() {
        Pneumatic.rightlatchSolenoid.set(true);
        Pneumatic.leftlatchSolenoid.set(true);
        
        robot.armPIDController.setReference(70, CANSparkMax.ControlType.kSmartMotion);

        if (Math.abs(robot.arm.getEncoder().getPosition() - 70) < 10) {
            currentState = State.DRIVE_TO_WALL;
        }
    }

    public void driveToWall() {
        final double LINE_ENCODER_COUNT = 5;

        //SmartDashboard.putNumber("STATE MACHINE RANDOM", new Random().nextDouble());
        //double yaw = robot.gyro.getAngle();

        // Drive forward and steer
        robot.DRIVE_LEFT.setReference(0.15, ControlType.kDutyCycle);
        robot.DRIVE_RIGHT.setReference(-0.15, ControlType.kDutyCycle);

        SmartDashboard.putNumber("drivetrain encoder distance", robot.DRIVE_LEFT_FRONT.getEncoder().getPosition());
        
        if (robot.DRIVE_LEFT_FRONT.getEncoder().getPosition() >= LINE_ENCODER_COUNT) {
            robot.DRIVE_LEFT.setReference(0, ControlType.kDutyCycle);
            robot.DRIVE_RIGHT.setReference(0, ControlType.kDutyCycle);
            currentState = State.SCORE_CONE;
        }
    }

    public void scoreCone() {
        Pneumatic.gripSolenoid.toggle();
        Pneumatic.pushSolenoid.toggle();

        currentState = State.DRIVE_TO_LINE;
    }
}
