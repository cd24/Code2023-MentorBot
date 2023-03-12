// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Autonomous.Auto;
import frc.robot.Constants.*;
import frc.robot.control.schemes.competition.*;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autoSelected;
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  private RobotContainer robot;
  private PowerDistribution pdp = new PowerDistribution();
  private Controllers controllers;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Configure the shared instance of our subsystems
    robot = new RobotContainer(
        new Drivetrain(new AHRS(Port.kMXP)),
        new Intake(WristConstants.intakeMotorCAN),
        new Wrist(WristConstants.wristMotor),
        new Arm(3, 1)
    );
    robot.engage();
    controllers = new Controllers(
      new DriverController(Constants.InputPorts.driverController, robot),
      new OperatorController(Constants.InputPorts.operatorController, robot)
      // Replace the line above to swap out the control scheme with a testing controller scheme defined
      // in the `IntakeIsolatedController.java` class!
      // new IntakeIsolatedController(Constants.InputPorts.operatorController, robot);
      //
      // (Also import it above) by replacing:
      // 
      // import frc.robot.control.schemes.competition.*;
      //
      // with the new import
      //
      // import frc.robot.control.schemes.competition.*;
    ).withBoundIO();
    
    pdp.clearStickyFaults();
    m_chooser.setDefaultOption("Default Auto (Move Arm)", robot.getAutonomousCommand(Auto.Selection.MOVEWRIST));
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
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    controllers.reportToSmartDashboard();
    SmartDashboard.putNumber("Port 21 Current", pdp.getCurrent(21));
  }

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
    pdp.clearStickyFaults();
    CommandScheduler.getInstance().schedule(m_chooser.getSelected());
    System.out.println("Auto selected: " + m_autoSelected);
    robot.getAutonomousCommand(Auto.Selection.MOVEWRIST);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // switch (m_autoSelected) {
    //   case kCustomAuto:
    //     // Put custom auto code here
    //     break;
    //   case kDefaultAuto:
    //   default:
    //     // Put default auto code here
    //     break;
    // }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
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
