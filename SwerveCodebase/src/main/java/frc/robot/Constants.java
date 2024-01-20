// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double DRIVETRAIN_LENGTH = 0.381;
  public static final double MAX_VOLTS = 4.95;

  public static final int MAX_SPEED = 1;

  public static final int DRIVETRAIN_LEFT_FRONT_SPARKMAX = 4;
  public static final int DRIVETRAIN_RIGHT_FRONT_SPARKMAX = 6;
  public static final int DRIVETRAIN_LEFT_BACK_SPARKMAX = 2;
  public static final int DRIVETRAIN_RIGHT_BACK_SPARKMAX = 7;

  public static final int DRIVETRAIN_LEFT_FRONT_SPARKMAX2 = 3;
  public static final int DRIVETRAIN_RIGHT_FRONT_SPARKMAX2 = 5;
  public static final int DRIVETRAIN_LEFT_BACK_SPARKMAX2 = 1;
  public static final int DRIVETRAIN_RIGHT_BACK_SPARKMAX2 = 8;

  public static final int DRIVETRAIN_ENCODER_BACK_LEFT = 9;
  public static final int DRIVETRAIN_ENCODER_FRONT_LEFT = 10;
  public static final int DRIVETRAIN_ENCODER_FRONT_RIGHT = 11;
  public static final int DRIVETRAIN_ENCODER_BACK_RIGHT = 12;

  // public static final double DRIVETRAIN_LEFT_FRONT_OFFSET = -0.093994;
  // public static final double DRIVETRAIN_RIGHT_FRONT_OFFSET = -0.385254;
  // public static final double DRIVETRAIN_LEFT_BACK_OFFSET = -0.490479;
  // public static final double DRIVETRAIN_RIGHT_BACK_OFFSET = -0.047852;

  public static final double DRIVETRAIN_LEFT_FRONT_OFFSET = -0;
  public static final double DRIVETRAIN_RIGHT_FRONT_OFFSET = -0;
  public static final double DRIVETRAIN_LEFT_BACK_OFFSET = -0;
  public static final double DRIVETRAIN_RIGHT_BACK_OFFSET = -0;


  public static final double ROBOT_LENGTH = 0.349;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
