// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;

import java.util.Random;

//import javax.security.auth.x500.X500Principal;

// importing some motors

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
//import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Declare arm motor
  //private final CANSparkMax armMotor = new CANSparkMax(34, MotorType.kBrushless);

  private final DriveTrain driveTrain = new DriveTrain(DriveTrain.DriveLayout.CYGNUS);

  double driveRight = 0;
  double driveLeft = 0;

  double pitchIAccumulator = 0;

  private final Joystick driverLeft = new Joystick(0);
  private final Joystick operator = new Joystick(2);

  boolean toggleState = false;
  boolean toggleLast = false; 

  // Declare Encoder
  ADIS16470_IMU gyro = new ADIS16470_IMU();

  // Decalring the lighting module.

  Spark lightController = new Spark(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

   // set arm motor run mode
   //SparkMaxPIDController setEncoder = armMotor.getPIDController();
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    Pneumatic.compressor.disable();

    
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
public void teleopInit() { /*Pneumatic.compressor.enableDigital();*/  Pneumatic.compressor.disable(); }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // Calls the state machine in the DriveTrain class
    if ( (driverLeft.getRawButton(12) == true) & (!toggleLast) ) {
      toggleState  = !toggleState;
    }

    toggleLast = driverLeft.getRawButton(12);
    driveTrain.tankDrive(toggleState);

    // Getting the arm up and going

    //armMotor.set(operator.getY());

    SmartDashboard.putNumber("gyro", operator.getY());

    Pneumatic.Solenoid.set(operator.getRawButton(1));

    // Setting a light preset using PWM

    lightController.set(0.53);


  }

  /** This function is called once when the robot is disabled. */
  @Override
public void disabledInit() { Pneumatic.compressor.disable(); }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    gyro.reset();
    gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kZ);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  /*
    //gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kX);
    double roll = gyro.getXComplementaryAngle();
    //gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kY);
    double yaw = gyro.getAngle();
    double pitch = gyro.getYComplementaryAngle();
    double pitchVel = gyro.getRate();
    
    SmartDashboard.putNumber("yaw", yaw);

    SmartDashboard.putNumber("pitch", pitch);
    SmartDashboard.putNumber("pitch vel", pitchVel);
    SmartDashboard.putNumber("roll", roll);

    pitchIAccumulator += pitch;

    double pitchD = 0.004;
    double pitchP = 0.005;
    double pitchI = 0.0001;
    double pitchDeadband = 0;

    double yawP = 0.01;


    
    if (Math.abs(gyro.getAngle()) > pitchDeadband) {
      driveTrain.driveLeftFront.set(-pitchP * pitch - pitchD * pitchVel - pitchI * pitchIAccumulator + yawP * yaw);
      driveTrain.driveRightFront.set(pitchP * pitch + pitchD * pitchVel + pitchI * pitchIAccumulator + yawP * yaw);
      driveTrain.driveLeftRear.set(-pitchP * pitch - pitchD * pitchVel - pitchI * pitchIAccumulator + yawP * yaw);
      driveTrain.driveRightRear.set(pitchP * pitch + pitchD * pitchVel + pitchI * pitchIAccumulator + yawP * yaw);
      
    }
    */
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
