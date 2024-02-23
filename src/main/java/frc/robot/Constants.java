// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ShooterConstants {
    public static final int TOP_MOTOR_ID = 12;
    public static final int BOTTOM_MOTOR_ID = 11;
    public static final int FEEDER_MOTOR_ID = 13;
    public static final double SHOOTER_SPEED = 0.66;
    public static final double FEEDER_SPEED = 0.20;
  }

  public static class IntakeConstants {
    public static final int LEFT_MOTOR_ID = 5;
    public static final int RIGHT_MOTOR_ID = 3;
    public static final int INTAKE_MOTOR_ID = 6;

    public static final double INTAKE_SPEED = 0.25;
    public static final double PIVOT_SPEED = 0.05;
    public static final double PIVOT_TOLERANCE = 2.5;

    public static final int PIVOT_DOWN_DEGREE = -90;
    public static final int PIVOT_UP_DEGREE = 0;
  }
}
