// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  public CANSparkMax leftFrontSparkMax1;
  public CANSparkMax rightFrontSparkMax1;
  public CANSparkMax leftBackSparkMax1;
  public CANSparkMax rightBackSparkMax1;

  public CANSparkMax leftFrontSparkMax2;
  public CANSparkMax rightFrontSparkMax2;
  public CANSparkMax leftBackSparkMax2;
  public CANSparkMax rightBackSparkMax2;

  public WheelDrive backRight;
  public WheelDrive backLeft;
  public WheelDrive frontRight;
  public WheelDrive frontLeft;

  public ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  public Drivetrain(WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft) {

    this.backLeft = backLeft;
    this.backRight = backRight;
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;

  }

  public Rotation2d getYaw() {
    return (false)
      ? Rotation2d.fromDegrees(360 - m_gyro.getAngle())
      : Rotation2d.fromDegrees(m_gyro.getAngle());
  }

  public void drive (double x1, double y1, double x2) {
    // double r = Math.sqrt ((Constants.DRIVETRAIN_LENGTH * Constants.DRIVETRAIN_LENGTH) + (Constants.DRIVETRAIN_LENGTH * Constants.DRIVETRAIN_LENGTH));
    // y1 *= -1;

    // double a = x1 - x2 * (Constants.DRIVETRAIN_LENGTH / r);
    // double b = x1 + x2 * (Constants.DRIVETRAIN_LENGTH / r);
    // double c = y1 - x2 * (Constants.DRIVETRAIN_LENGTH / r);
    // double d = y1 + x2 * (Constants.DRIVETRAIN_LENGTH / r);

    // double backRightSpeed = Math.sqrt ((a * a) + (d * d));
    // double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
    // double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
    // double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

    // double backRightAngle = Math.atan2 (a, d) / Math.PI;
    // double backLeftAngle = Math.atan2 (a, c) / Math.PI;
    // double frontRightAngle = Math.atan2 (b, d) / Math.PI;
    // double frontLeftAngle = Math.atan2 (b, c) / Math.PI;

    Translation2d m_frontLeftLocation = new Translation2d(Constants.ROBOT_LENGTH, Constants.ROBOT_LENGTH);
    Translation2d m_frontRightLocation = new Translation2d(Constants.ROBOT_LENGTH, -Constants.ROBOT_LENGTH);
    Translation2d m_backLeftLocation = new Translation2d(-Constants.ROBOT_LENGTH, Constants.ROBOT_LENGTH);
    Translation2d m_backRightLocation = new Translation2d(-Constants.ROBOT_LENGTH, -Constants.ROBOT_LENGTH);

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    

    ChassisSpeeds speeds = new ChassisSpeeds(x1, y1, x2);

    SmartDashboard.putNumber("x", x1);
    SmartDashboard.putNumber("y", y1);
    SmartDashboard.putNumber("z", x2);

    // Convert to module states
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
    // SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getYaw(),new Pose2d());

    // Front left module state
    SwerveModuleState frontLeftState = moduleStates[0];
   frontLeftState = SwerveModuleState.optimize(frontLeftState, 
    new Rotation2d((frontLeft.encoder_wheel.getAbsolutePosition().getValueAsDouble()-Constants.DRIVETRAIN_LEFT_FRONT_OFFSET)*2*Math.PI));

    // Front right module state
    SwerveModuleState frontRightState = moduleStates[1];
    frontRightState = SwerveModuleState.optimize(frontRightState,
    new Rotation2d((frontRight.encoder_wheel.getAbsolutePosition().getValueAsDouble()-Constants.DRIVETRAIN_RIGHT_FRONT_OFFSET)*2*Math.PI));

    // Back left module state
    SwerveModuleState backLeftState = moduleStates[2];
    backLeftState = SwerveModuleState.optimize(backLeftState,
    new Rotation2d((backLeft.encoder_wheel.getAbsolutePosition().getValueAsDouble()-Constants.DRIVETRAIN_LEFT_BACK_OFFSET)*2*Math.PI));

    // Back right module state
    SwerveModuleState backRightState = moduleStates[3];
    backRightState = SwerveModuleState.optimize(backRightState,
    new Rotation2d((backRight.encoder_wheel.getAbsolutePosition().getValueAsDouble()-Constants.DRIVETRAIN_RIGHT_BACK_OFFSET)*2*Math.PI));


    backRight.drive (backRightState.speedMetersPerSecond, backRightState.angle, Constants.DRIVETRAIN_ENCODER_BACK_RIGHT);
    //SmartDashboard.putNumber("Encoder Position BackRight", backRight.encoder_wheel.getAbsolutePosition().getValueAsDouble()-Constants.DRIVETRAIN_RIGHT_BACK_OFFSET);
    SmartDashboard.putNumber("Encoder Position BackRight", backRight.encoder_position);
    SmartDashboard.putNumber("Back Right Expected Angle", backRightState.angle.getDegrees());
    SmartDashboard.putNumber("Speed Back Right", backRightState.speedMetersPerSecond);
    SmartDashboard.putNumber("Back Right PID Calc", backRight.PIDSpeed);
    
    backLeft.drive (backLeftState.speedMetersPerSecond, backLeftState.angle, Constants.DRIVETRAIN_ENCODER_BACK_LEFT);
    //SmartDashboard.putNumber("Encoder Position BackLeft",backLeft.encoder_wheel.getAbsolutePosition().getValueAsDouble()-Constants.DRIVETRAIN_LEFT_BACK_OFFSET);
    SmartDashboard.putNumber("Encoder Position BackLeft",backLeft.encoder_position);
    SmartDashboard.putNumber("Back Left Expected Angle", backLeftState.angle.getDegrees());
    SmartDashboard.putNumber("Speed Left State", backLeftState.speedMetersPerSecond);
    SmartDashboard.putNumber("Back Left PID Calc", backLeft.PIDSpeed);

    frontRight.drive (frontRightState.speedMetersPerSecond, frontRightState.angle, Constants.DRIVETRAIN_ENCODER_FRONT_RIGHT);
    //SmartDashboard.putNumber("Encoder Position FrontRight", frontRight.encoder_wheel.getAbsolutePosition().getValueAsDouble()-Constants.DRIVETRAIN_RIGHT_FRONT_OFFSET);
    SmartDashboard.putNumber("Encoder Position FrontRight", frontRight.encoder_position);
    SmartDashboard.putNumber("Front Right Expected Angle", frontRightState.angle.getDegrees());
    SmartDashboard.putNumber("Speed Front Right", frontRightState.speedMetersPerSecond);
    SmartDashboard.putNumber("Front Right PID Calc", frontRight.PIDSpeed);
    
    frontLeft.drive (frontLeftState.speedMetersPerSecond, frontLeftState.angle, Constants.DRIVETRAIN_ENCODER_FRONT_LEFT);
    //SmartDashboard.putNumber("Encoder Position FrontLeft", frontLeft.encoder_wheel.getAbsolutePosition().getValueAsDouble()-Constants.DRIVETRAIN_LEFT_FRONT_OFFSET);
    SmartDashboard.putNumber("Encoder Position FrontLeft", frontLeft.encoder_position);
    SmartDashboard.putNumber("Front Left Expected Angle", frontLeftState.angle.getDegrees());
    SmartDashboard.putNumber("Speed Front Left", frontLeftState.speedMetersPerSecond);
    SmartDashboard.putNumber("Front Left PID Calc", frontLeft.PIDSpeed);

    SmartDashboard.putNumber("Setpoint BackRight", backRight.setpoint);
    SmartDashboard.putNumber("Setpoint BackLeft",backLeft.setpoint);
    SmartDashboard.putNumber("Setpoint FrontRight", frontRight.setpoint);
    SmartDashboard.putNumber("Setpoint FrontLeft", frontLeft.setpoint);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
