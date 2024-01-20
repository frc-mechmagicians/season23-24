// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.lang.invoke.ConstantBootstraps;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WheelDrive extends SubsystemBase {
  /** Creates a new WheelDrive. */

  private CANSparkMax angleMotor;
  private CANSparkMax speedMotor;
  private PIDController pidController;
  public int encoderID;
  public double setpoint;
  public CANcoder encoder_wheel = new CANcoder(this.encoderID);
  public double offset;
  public double encoder_position;
  public double PIDSpeed;

  public WheelDrive(int angleMotor, int speedMotor, int portNumberA, double offset) {

    this.angleMotor = new CANSparkMax(angleMotor, MotorType.kBrushless);

    this.speedMotor = new CANSparkMax (speedMotor, MotorType.kBrushless);
    pidController = new PIDController (SmartDashboard.getNumber("kP", 0.1), SmartDashboard.getNumber("kI", 0), SmartDashboard.getNumber("kD", 0));
    pidController.setTolerance(.3/360);
    pidController.enableContinuousInput(0,1);
    this.encoderID = portNumberA;
    encoder_wheel = new CANcoder(this.encoderID);
    this.offset = offset;
  }

  public void drive (double speed, Rotation2d angle, int encoder) {

    //speedMotor.set(0.1);

    setpoint = angle.getDegrees();
    if (setpoint < 0) {
        setpoint += 360;
    } 

    setpoint %= 360;

    setpoint/=360;



    //     this.angleMotor.set(pidController.calculate(encoder_wheel.getAbsolutePosition().getValueAsDouble()-offset, setpoint+.125));


    encoder_position = encoder_wheel.getAbsolutePosition().getValueAsDouble()-offset;


    // if (!(this.offset == Constants.DRIVETRAIN_LEFT_BACK_OFFSET)) {
    
    if (encoder_position < 0) {
      encoder_position += 1;
    }

    encoder_position %= 1;

    pidController.setSetpoint(setpoint);

    pidController.setP(SmartDashboard.getNumber("kP", 0.1));
    pidController.setI(SmartDashboard.getNumber("kI", 0));
    pidController.setD(SmartDashboard.getNumber("kD", 0));


    // if (!(pidController.atSetpoint())) {
    this.angleMotor.setInverted(true); 

    PIDSpeed = pidController.calculate(encoder_position, setpoint);
    this.angleMotor.set(PIDSpeed);
    //this.angleMotor.set(0.075);
    // } else {
    //   this.angleMotor.set(0);
    // }

    //this.angleMotor.set(0.1);
    // if (this.angleMotor.getDeviceId() == Constants.DRIVETRAIN_LEFT_BACK_SPARKMAX) {
    //   speedMotor.setInverted(false);
    // }
    // if (this.angleMotor.getDeviceId() == Constants.DRIVETRAIN_RIGHT_FRONT_SPARKMAX) {
    //   speedMotor.setInverted(false);
    // }

    speedMotor.setInverted(true);

    if (speedMotor.getDeviceId() == Constants.DRIVETRAIN_RIGHT_BACK_SPARKMAX2) {
      speedMotor.setInverted(false);
    }

    speedMotor.set(speed/Math.sqrt(2));
    //speedMotor.set(.1);
    

  }

  public void reset() {
    this.angleMotor.set(pidController.calculate(encoder_wheel.getAbsolutePosition().getValueAsDouble(), 0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
