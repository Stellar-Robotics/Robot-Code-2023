// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import pabeles.concurrency.IntOperatorTask.Max;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  // Define Motors.
  private final CANSparkMax driveLeftFront = new CANSparkMax(32, MotorType.kBrushless);
  private final CANSparkMax driveRightFront = new CANSparkMax(30, MotorType.kBrushless);
  private final CANSparkMax driveLeftRear = new CANSparkMax(33, MotorType.kBrushless);
  private final CANSparkMax driveRightRear = new CANSparkMax(31, MotorType.kBrushless);
  // Define Controllers
  private final Joystick driverLeft = new Joystick(0);
  private final Joystick driverRight = new Joystick(1);
  private final Joystick Operator = new Joystick(2);

  double driveRight = 0;
  double driveLeft = 0;

  boolean toggleState = false;
  boolean toggleLast = false; 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
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
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Setting a multiplier that is changed when the trigger on joystic is held down.

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
        driveLeft = driverRight.getZ() * driveSpeedMultiplier * 0.;
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

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}