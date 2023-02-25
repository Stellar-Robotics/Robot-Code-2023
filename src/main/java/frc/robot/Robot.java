// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Import necessary libraries

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {

  public enum DriveLayout {
    CYGNUS,
    NEW
  }

  // Set drive mode
  private static DriveLayout driveLayout = DriveLayout.NEW;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Declare inputs.

  private final Joystick DRIVER_LEFT_JOYSTICK = new Joystick(0);
  private final Joystick DRIVER_RIGHT_JOYSTICK = new Joystick(1);
  private final Joystick operator = new Joystick(2);
  private final Joystick guitar = new Joystick(3);
  private final XboxController xOperator = new XboxController(4);

  // Declare motors & actuators.
  public final CANSparkMax DRIVE_LEFT_FRONT;
  public final CANSparkMax DRIVE_RIGHT_FRONT;
  public final CANSparkMax DRIVE_LEFT_REAR;
  public final CANSparkMax DRIVE_RIGHT_REAR;

  public final MotorControllerGroup DRIVE_LEFT;
  public final MotorControllerGroup DRIVE_RIGHT;


  private DifferentialDrive drivetrain;

  // Setting variables for logic.

  private final double MAX_ARM_HEIGHT = 75;

  double armP = 0;
  double armI = 0;
  double armD = 0;

  double driveRight = 0;
  double driveLeft = 0;

  double pitchIAccumulator = 0;

  boolean toggleState = false;
  boolean toggleLast = false;
  
  boolean toggleLastArm = false;

  double targetPosition = 0;

  // Actuator and Gyro Declaration

  ADIS16470_IMU gyro = new ADIS16470_IMU(); // Creating the gyro refrence.

  static CANSparkMax arm = new CANSparkMax(34, MotorType.kBrushless); // Creating the Arm Motor refrence.

  static RelativeEncoder armEncoder = arm.getEncoder(); // Getting the encoder for the arm

  SparkMaxPIDController armPIDController = arm.getPIDController(); // Creating a PID controller.

  Spark lightController = new Spark(0); // Decalring the lighting module.

  public Robot() {
    super();

    if (driveLayout == DriveLayout.NEW) {
      DRIVE_LEFT_FRONT = new CANSparkMax(32, MotorType.kBrushless);
      DRIVE_RIGHT_FRONT = new CANSparkMax(30, MotorType.kBrushless);
      DRIVE_LEFT_REAR = new CANSparkMax(33, MotorType.kBrushless);
      DRIVE_RIGHT_REAR = new CANSparkMax(31, MotorType.kBrushless);
    } else {
      DRIVE_LEFT_FRONT = new CANSparkMax(50, MotorType.kBrushless);
      DRIVE_RIGHT_FRONT = new CANSparkMax(54, MotorType.kBrushless);
      DRIVE_LEFT_REAR = new CANSparkMax(42, MotorType.kBrushless);
      DRIVE_RIGHT_REAR = new CANSparkMax(52, MotorType.kBrushless);
    }

    DRIVE_LEFT = new MotorControllerGroup(DRIVE_LEFT_FRONT, DRIVE_LEFT_REAR);
    DRIVE_RIGHT = new MotorControllerGroup(DRIVE_RIGHT_FRONT, DRIVE_RIGHT_REAR);

    drivetrain = new DifferentialDrive(DRIVE_LEFT, DRIVE_RIGHT);

    // Set up smart motion constraints for arm
    armPIDController.setSmartMotionMaxVelocity(0.2, 0);
    armPIDController.setSmartMotionMaxAccel(0.1, 0);
    
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  // Initialization code to be run after robot startup

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Declaring telemetry for SmartDashboard
        
    SmartDashboard.putNumber("Pitch", 0);
    SmartDashboard.putNumber("Yaw", 0);
    SmartDashboard.putNumber("Pitch Velocity", 0);
    SmartDashboard.putNumber("P", 0.0045);
    SmartDashboard.putNumber("I", 0.00001);
    SmartDashboard.putNumber("D", 0.006);
    SmartDashboard.putNumber("PitchMaxDVelocity", 2.5);
    SmartDashboard.putNumber("StopAngle", 4);

    SmartDashboard.putNumber("ArmP", 0.01);
    SmartDashboard.putNumber("ArmI", 0); 
    SmartDashboard.putNumber("ArmD", 0); 
    SmartDashboard.putNumber("ArmPos", 0);

    SmartDashboard.putNumber("TestButtonSpike", 0);
    SmartDashboard.putBoolean("DriveSpeed", false);

    // Setting hardware device values

    Pneumatic.compressor.disable();
    lightController.set(0.69);



  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() { lightController.set(0.69); }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() { Pneumatic.compressor.enableDigital(); }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // Calls the state machine in the DriveTrain class

    if ( (DRIVER_LEFT_JOYSTICK.getRawButton(12) == true) & (!toggleLast) ) {

      toggleState  = !toggleState;

    }
    toggleLast = DRIVER_LEFT_JOYSTICK.getRawButton(12);

    // Calculate multiplier
    double driveSpeedMultiplier = 1;
    if (toggleState || armEncoder.getPosition() > 5) {
      driveSpeedMultiplier *= 0.5;
    }

    // Calculate power based on power and multiplier
    double driveLeftPower = -DRIVER_LEFT_JOYSTICK.getY() * driveSpeedMultiplier;
    double driveRightPower = DRIVER_RIGHT_JOYSTICK.getY() * driveSpeedMultiplier;

    driveRightPower += 0.005;

    // Apply motor power
    drivetrain.tankDrive(driveLeftPower, driveRightPower);


    // Pneumatic Actuation Code
    
    if (operator.getRawButtonPressed(3)) {

      Pneumatic.gripSolenoid.toggle();

    }

    //toggleLastArm = guitar.getRawButton(5);

    if (operator.getRawButtonPressed(4)){

      Pneumatic.pushSolenoid.toggle();

    }

    // Setting a light preset using PWM

    lightController.set(0.37);

    // Arm Positioning and PID

    
    if ( operator.getY() < -0.25 && targetPosition <= 85 && operator.getRawButton(1)) {

      targetPosition += 0.5;

    }

    else if ( operator.getY() > 0.25 && targetPosition >= -14 && operator.getRawButton(1)) {

      targetPosition -= 0.5;

    } 

    //targetPosition = operator.getY() * MAX_ARM_HEIGHT;

    armPIDController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);

    // Get PID values.
    double newArmP = SmartDashboard.getNumber("ArmP", 0.1);
    double newArmI = SmartDashboard.getNumber("ArmI", 0);
    double newArmD = SmartDashboard.getNumber("ArmD", 0);

    // If the values have changed, update the controller.
    // (It's not good to constantly update PID values, even if they stay the same)
    if (newArmP != armP || newArmI != armI || newArmD != armD) {
      armP = SmartDashboard.getNumber("ArmP", 0.1);
      armI = SmartDashboard.getNumber("ArmI", 0);
      armD = SmartDashboard.getNumber("ArmD", 0);

      armPIDController.setP(armP);
      armPIDController.setI(armI);
      armPIDController.setD(armD);
    }

    // Update SmartDashboard
    SmartDashboard.putNumber("TestButtonSpike", guitar.getRawButton(5)? 1 : 0);
    SmartDashboard.putNumber("Encoder", armEncoder.getPosition());
    SmartDashboard.putNumber("GuitarHat", guitar.getPOV());
    SmartDashboard.putNumber("OperatorY", operator.getY());
    SmartDashboard.putBoolean("DriveSpeed", (toggleState || armEncoder.getPosition() > 5));

  }

  /** This function is called once when the robot is disabled. */
  @Override
public void disabledInit() { Pneumatic.compressor.disable(); }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    pitchIAccumulator = 0;
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    gyro.reset();
    gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kZ);

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  
    double roll = gyro.getXComplementaryAngle();
    double yaw = gyro.getAngle();
    double pitch = -gyro.getYComplementaryAngle();
    double pitchVel = -gyro.getRate();

    double pitchIReset = 12;
    
    SmartDashboard.putNumber("Yaw", yaw);

    SmartDashboard.putNumber("Pitch", pitch);
    SmartDashboard.putNumber("Pitch Velocity", pitchVel);
    SmartDashboard.putNumber("roll", roll);

    pitchIAccumulator = Math.abs(pitch) > pitchIReset? pitchIAccumulator + pitch: 0;

    SmartDashboard.putNumber("Accumulated I", pitchIAccumulator);


    double pitchD = SmartDashboard.getNumber("D", 0);
    double pitchP = SmartDashboard.getNumber("P", 0);
    double pitchI = SmartDashboard.getNumber("I", 0);

    //double stopAngle = SmartDashboard.getNumber("StopAngle", 4);

    double pitchMaxDVelocity = SmartDashboard.getNumber("PitchMaxDVelocity", 100);
    
    

    double yawP = 0.005;

    double pitchForceP = (pitchP * Math.pow(pitch, 2));
    double pitchForceI = (pitchI * pitchIAccumulator);
    double pitchForceD = (pitchD * (Math.abs(pitchVel) < pitchMaxDVelocity? pitchVel : 0));
    
    

    double pitchForce = pitchForceP + pitchForceI + pitchForceD;

    double driveLeftPower =  -pitchForce + (yawP * yaw);
    double driveRightPower = pitchForce + (yawP * yaw);
    
    SmartDashboard.putNumber("power", pitchForce);

    drivetrain.tankDrive(driveLeftPower, driveRightPower);


      

    
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
