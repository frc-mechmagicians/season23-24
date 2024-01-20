// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.WheelDrive;
import com.ctre.phoenix6.hardware.CANcoder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private WheelDrive backLeft = new WheelDrive (Constants.DRIVETRAIN_LEFT_BACK_SPARKMAX, Constants.DRIVETRAIN_LEFT_BACK_SPARKMAX2, Constants.DRIVETRAIN_ENCODER_BACK_LEFT,Constants.DRIVETRAIN_LEFT_BACK_OFFSET);
  private WheelDrive frontLeft = new WheelDrive (Constants.DRIVETRAIN_LEFT_FRONT_SPARKMAX, Constants.DRIVETRAIN_LEFT_FRONT_SPARKMAX2, Constants.DRIVETRAIN_ENCODER_FRONT_LEFT,Constants.DRIVETRAIN_LEFT_FRONT_OFFSET);
  private WheelDrive frontRight = new WheelDrive (Constants.DRIVETRAIN_RIGHT_FRONT_SPARKMAX, Constants.DRIVETRAIN_RIGHT_FRONT_SPARKMAX2, Constants.DRIVETRAIN_ENCODER_FRONT_RIGHT,Constants.DRIVETRAIN_RIGHT_FRONT_OFFSET);
  private WheelDrive backRight = new WheelDrive (Constants.DRIVETRAIN_RIGHT_BACK_SPARKMAX, Constants.DRIVETRAIN_RIGHT_BACK_SPARKMAX2, Constants.DRIVETRAIN_ENCODER_BACK_RIGHT,Constants.DRIVETRAIN_RIGHT_BACK_OFFSET);

  private Drivetrain swerveDrive = new Drivetrain (backRight, backLeft, frontRight, frontLeft);

  private PS4Controller joystick = new PS4Controller(0);

  private RobotContainer m_robotContainer;

  private int timer = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    backLeft.reset();
    backRight.reset();
    frontLeft.reset();
    frontRight.reset();

    SmartDashboard.putNumber("Encoder Position BackRight", backRight.encoder_wheel.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("Encoder Position backLeft", backLeft.encoder_wheel.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("Encoder Position frontRight", frontRight.encoder_wheel.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("Encoder Position frontLeft", frontLeft.encoder_wheel.getAbsolutePosition().getValueAsDouble());


  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.  
    
    // CANcoder encoderWheel = new CANcoder(Constants.DRIVETRAIN_ENCODER_BACK_LEFT);
    // encoderWheel.setPosition(0);
    // encoderWheel = new CANcoder(Constants.DRIVETRAIN_ENCODER_BACK_RIGHT);
    // encoderWheel.setPosition(0);
    // encoderWheel = new CANcoder(Constants.DRIVETRAIN_ENCODER_FRONT_LEFT);
    // encoderWheel.setPosition(0);
    // encoderWheel = new CANcoder(Constants.DRIVETRAIN_ENCODER_FRONT_RIGHT);
    // encoderWheel.setPosition(0);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double x1 = (joystick.getRawAxis(0));
    double y1 = (joystick.getRawAxis(1));
    double x2 = (joystick.getRawAxis(4));

    if (Math.abs(x1) < 0.01) {
      x1 = 0;
    }
    if (Math.abs(y1) < 0.01) {
      y1 = 0;
    }
    if (Math.abs(x2) < 0.01) {
      x2 = 0;
    }


    swerveDrive.drive(-y1/10, -x1/10, -x2/10);

    //swerveDrive.drive(0, 1, 0);

    //swerveDrive.drive (0.1, 0, 0);

    //Change
    
    // if (timer == 250) {
    //   swerveDrive.drive (0.1, 0, 0);
    // } 
    // if (timer == 500) {
    //   swerveDrive.drive (0, 0.1, 0);
    // } 
    // if (timer == 750) {
    //   swerveDrive.drive (-0.1, 0, 0);
    // } 
    // if (timer == 1000) {
    //   swerveDrive.drive (0, -0.1, 0);
    //   timer = 0;
    // } 

    SmartDashboard.putNumber("Up and Down", joystick.getRawAxis (0));
    SmartDashboard.putNumber("Left Right", joystick.getRawAxis (1));
    SmartDashboard.putNumber("Rotate", joystick.getRawAxis (4));

    timer++;

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

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
