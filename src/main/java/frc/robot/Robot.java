// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Import necessary libraries

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutonomousStateMachine.State;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

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
  public static DriveLayout driveLayout = DriveLayout.NEW;

  public static final String kDefaultAuto = "Default";
  public static final String kCustomAuto = "My Auto";
  public String m_autoSelected;
  public final SendableChooser<String> m_chooser = new SendableChooser<>();

  // The amount of power to reduce to the left side of the drive train
  public final double SERENOCITY = 0.85;

  // Declare inputs.

  public final Joystick DRIVER_LEFT_JOYSTICK = new Joystick(0);
  public final Joystick DRIVER_RIGHT_JOYSTICK = new Joystick(1);
  public final Joystick OPERATOR = new Joystick(2);
  public final GenericHID BUTTON_PANEL = new GenericHID(3);

  // Declare motors & actuators.
  public final CANSparkMax DRIVE_LEFT_FRONT;
  public final CANSparkMax DRIVE_RIGHT_FRONT;
  public final CANSparkMax DRIVE_LEFT_REAR;
  public final CANSparkMax DRIVE_RIGHT_REAR;
  public final CANSparkMax ROLLER;

  public final SparkMaxPIDController DRIVE_LEFT;
  public final SparkMaxPIDController DRIVE_RIGHT;

  // Setting variables for logic.

  public final double MAX_ARM_HEIGHT = 80;

  double armP = 0;
  double armI = 0;
  double armD = 0;

  double driveRight = 0;
  double driveLeft = 0;

  double pitchIAccumulator = 0;

  boolean toggleState = false;
  boolean toggleLast = false;
  boolean flipDrivetrain = false;
  
  boolean toggleLastArm = false;

  double targetPosition = 0;

  // Actuator and Gyro Declaration

  ADIS16470_IMU gyro = new ADIS16470_IMU(); // Creating the gyro refrence.

  static CANSparkMax arm = new CANSparkMax(34, MotorType.kBrushless); // Creating the Arm Motor refrence.
  
  static RelativeEncoder armEncoder = arm.getEncoder(); // Getting the encoder for the arm

  SparkMaxPIDController armPIDController = arm.getPIDController(); // Creating a PID controller.

  Spark lightController = new Spark(0); // Decalring the lighting module.

  // AUTO STUFF
  AutonomousStateMachine auto;
  int selectedAutoMode;
  enum AutoMode {
    DRIVE_AND_BALANCE,
    DO_NOTHING,
    DRIVE_TO_LINE,
    SCORE_CONE,
    SCORE_CONE_GRAB_ELEM
  }

  // CAMERA STUFF
  UsbCamera camera;

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

    ROLLER = new CANSparkMax(36, MotorType.kBrushless);

    DRIVE_LEFT_REAR.follow(DRIVE_LEFT_FRONT);
    DRIVE_RIGHT_REAR.follow(DRIVE_RIGHT_FRONT);

    DRIVE_LEFT = DRIVE_LEFT_FRONT.getPIDController();
    DRIVE_RIGHT = DRIVE_RIGHT_FRONT.getPIDController();

    // Set up smart motion constraints for drivetrain
    //DRIVE_LEFT.setSmartMotionMaxAccel(kDefaultPeriod, selectedAutoMode)
    //DRIVE_LEFT.setP(0.001);
    //DRIVE_RIGHT.setP(0.001);


    // Set up smart motion constraints for arm
    armPIDController.setSmartMotionMaxVelocity(5820, 0);
    armPIDController.setSmartMotionMaxAccel(4000, 0); 
    armPIDController.setSmartMotionAllowedClosedLoopError(2, 0);
    
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
    SmartDashboard.putNumber("P", 0.0008);
    SmartDashboard.putNumber("I", 0.000);
    SmartDashboard.putNumber("D", 0.00004);

    SmartDashboard.putNumber("ArmP", 0.0008);
    SmartDashboard.putNumber("ArmI", 0); 
    SmartDashboard.putNumber("ArmD", 0.0001); 
    SmartDashboard.putNumber("ArmPos", 0);

    SmartDashboard.putNumber("PitchP", 20);
    SmartDashboard.putNumber("PitchI", 0);
    SmartDashboard.putNumber("PitchD", 12);

    SmartDashboard.putBoolean("DriveSpeed", false);

    SmartDashboard.putNumber("ConeColor", 0.63);
    SmartDashboard.putNumber("CubeColor", 0.57);

    // Setting hardware device values

    Pneumatic.compressor.disable();

    CameraServer.startAutomaticCapture();


    Pneumatic.phub.clearStickyFaults();
    PowerDistribution pdp = new PowerDistribution();
    pdp.clearStickyFaults();
    pdp.close();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
    gyro.reset();
    gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kZ);

    switch (AutoMode.values()[selectedAutoMode]) {
        case DO_NOTHING: {auto = new AutonomousStateMachine(this, State.DO_NOTHING); break;}
        case DRIVE_AND_BALANCE: {auto = new AutonomousStateMachine(this, State.DRIVE_TO_PLATFORM); break;}
        case DRIVE_TO_LINE: {auto = new AutonomousStateMachine(this, State.DRIVE_TO_LINE); break;}
        case SCORE_CONE: {auto = new AutonomousStateMachine(this, State.RAISE_ARM); break;}
        case SCORE_CONE_GRAB_ELEM: {auto = new AutonomousStateMachine(this, State.RAISE_ARM); 
          auto.getElement = true;
          break;}    
      }

    double P = SmartDashboard.getNumber("P", 0);
    if (P != DRIVE_LEFT.getP()) {
      DRIVE_LEFT.setP(P);
      DRIVE_RIGHT.setP(P);
    }
    SmartDashboard.putNumber("P", DRIVE_LEFT.getP());

    double I = SmartDashboard.getNumber("I", 0);
    if (I != DRIVE_LEFT.getI()) {
      DRIVE_LEFT.setI(I);
      DRIVE_RIGHT.setI(I);
    }
    SmartDashboard.putNumber("I", DRIVE_LEFT.getI());

    double D = SmartDashboard.getNumber("D", 0);
    if (D != DRIVE_LEFT.getD()) {
      DRIVE_LEFT.setD(D);
      DRIVE_RIGHT.setD(D);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    auto.run();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() { 
    Pneumatic.compressor.enableDigital(); 
  }

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
      driveSpeedMultiplier *= 0.55;
    }

    if (DRIVER_RIGHT_JOYSTICK.getRawButton(1)) {
      double driveLeftPower = DRIVER_RIGHT_JOYSTICK.getZ() * driveSpeedMultiplier * 0.6;
      double driveRightPower = DRIVER_RIGHT_JOYSTICK.getZ() * driveSpeedMultiplier * 0.6;

      // Apply motor power
      DRIVE_LEFT.setReference(driveLeftPower, ControlType.kDutyCycle);
      DRIVE_RIGHT.setReference(driveRightPower, ControlType.kDutyCycle);
    }
    else {
      // Calculate power based on power and multiplier
      double driveLeftPower = -DRIVER_LEFT_JOYSTICK.getY() * driveSpeedMultiplier;
      double driveRightPower = (DRIVER_RIGHT_JOYSTICK.getY() * SERENOCITY) * driveSpeedMultiplier;

      // Apply motor power

      if (flipDrivetrain) {
        DRIVE_LEFT.setReference(driveRightPower, ControlType.kDutyCycle);
        DRIVE_RIGHT.setReference(driveLeftPower, ControlType.kDutyCycle);
      } else {
        DRIVE_LEFT.setReference(driveLeftPower, ControlType.kDutyCycle);
        DRIVE_RIGHT.setReference(driveRightPower, ControlType.kDutyCycle);
      }

    }

    if (DRIVER_LEFT_JOYSTICK.getRawButtonPressed(11)) {
      flipDrivetrain = !flipDrivetrain;
    }

    if (OPERATOR.getRawButton(7)) {

      ROLLER.set(0.5);

      Pneumatic.pushSolenoid.set(true);

    }

    else if (OPERATOR.getRawButton(8)) {

      ROLLER.set(-0.5);

      Pneumatic.pushSolenoid.set(true);

    }

    else {

      ROLLER.set(0);

    }

    // Pneumatic Actuation Code
    
    if (OPERATOR.getRawButtonPressed(4)) {

      Pneumatic.pushSolenoid.toggle();

    }

    
    if (armEncoder.getPosition() < 45) {

      Pneumatic.gripSolenoid.set(false);

    }

    else if (OPERATOR.getRawButtonPressed(3)) {

      Pneumatic.gripSolenoid.toggle();

    }

    OPERATOR.getRawButtonPressed(3);

    //Operator LED contorl for Human Player comunication

    if (OPERATOR.getRawButtonPressed(5)){
      lightController.set(SmartDashboard.getNumber("ConeColor", 0.63));
      //double setTime = System.currentTimeMillis();
    }
    if (OPERATOR.getRawButtonPressed(6)){
      lightController.set(SmartDashboard.getNumber("CubeColor", 0.57));
      //double setTime = System.currentTimeMillis();
    }
    if (OPERATOR.getRawButtonPressed(2)){
      lightController.set(SmartDashboard.getNumber("Default Color", 0.87));
      //double setTime = System.currentTimeMillis();
    }
    
    //if (lightController.get() == 0.17 && (System.currentTimeMillis() - 6) > 50) {
      //lightController.set(0.17);
    //}
    
    //lightController.set(0.65);
    // Setting a light preset using PWM

    //lightController.set(0.37);

    // Arm Positioning and PID
    if ( OPERATOR.getY() < -0.25 && targetPosition <= 90 && OPERATOR.getRawButton(1)) {

      targetPosition += 1;

    } else if ( OPERATOR.getY() > 0.25 && targetPosition >= 0 && OPERATOR.getRawButton(1)) {

      targetPosition -= 2.5;

    }

    if (OPERATOR.getPOV() == 0) {
      targetPosition = 70;
    } else if (OPERATOR.getPOV() == 180) {
      targetPosition = 60;
    }
  
    Pneumatic.rightlatchSolenoid.set(targetPosition > 0 || OPERATOR.getRawButton(9));
    Pneumatic.leftlatchSolenoid.set(targetPosition > 0 || OPERATOR.getRawButton(9));

    //targetPosition = Math.max(0, -operator.getY() * MAX_ARM_HEIGHT);
    
    SmartDashboard.putNumber("armPosition", targetPosition);
    armPIDController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion);

    // Get PID values.
    double newArmP = SmartDashboard.getNumber("ArmP", 0.1);
    double newArmI = SmartDashboard.getNumber("ArmI", 0);
    double newArmD = SmartDashboard.getNumber("ArmD", 0);

    // If the values have changed, update the controller.
    // (It's not good to constantly update PID values, even if they stay the same)
    if (newArmP != armP || newArmI != armI || newArmD != armD) {
      armP = SmartDashboard.getNumber("ArmP", 0.05);
      armI = SmartDashboard.getNumber("ArmI", 0);
      armD = SmartDashboard.getNumber("ArmD", 0);

      armPIDController.setP(armP);
      armPIDController.setI(armI);
      armPIDController.setD(armD);
    }

    // Update SmartDashboard
    SmartDashboard.putNumber("Encoder", armEncoder.getPosition());
    SmartDashboard.putNumber("OperatorY", OPERATOR.getY());
    SmartDashboard.putBoolean("DriveSpeed", (toggleState || armEncoder.getPosition() > 5));
    
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() { Pneumatic.compressor.disable(); }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    if (SmartDashboard.getBoolean("Next Auto", true)) {
      selectedAutoMode = (selectedAutoMode + 1) % AutoMode.values().length;
      SmartDashboard.putBoolean("Next Auto", false);
    }
    
    SmartDashboard.putString("Selected Auto", AutoMode.values()[selectedAutoMode].toString());
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    final double targetVelocity = 480;

    double P = SmartDashboard.getNumber("P", 0);
    if (P != DRIVE_LEFT.getP()) {
      DRIVE_LEFT.setP(P);
      DRIVE_RIGHT.setP(P);
    }
    SmartDashboard.putNumber("P", DRIVE_LEFT.getP());

    double I = SmartDashboard.getNumber("I", 0);
    if (I != DRIVE_LEFT.getI()) {
      DRIVE_LEFT.setI(I);
      DRIVE_RIGHT.setI(I);
    }
    SmartDashboard.putNumber("I", DRIVE_LEFT.getI());

    double D = SmartDashboard.getNumber("D", 0);
    if (D != DRIVE_LEFT.getD()) {
      DRIVE_LEFT.setD(D);
      DRIVE_RIGHT.setD(D);
    }
    SmartDashboard.putNumber("D", DRIVE_LEFT.getD());

    DRIVE_LEFT.setReference(-targetVelocity, CANSparkMax.ControlType.kVelocity);
    DRIVE_RIGHT.setReference(targetVelocity, CANSparkMax.ControlType.kVelocity);

    SmartDashboard.putNumber("DriveVelocity", DRIVE_LEFT_FRONT.getEncoder().getVelocity());
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
