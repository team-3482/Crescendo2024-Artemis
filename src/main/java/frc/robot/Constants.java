// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  // Constants for the controller and any controller related items
  // (ex. buttons, axis, ect.)
  public final static class ControllerConstants {
    public static int DRIVE_CONTROLLER_ID = 0;

    public static int DRIVE_ROT_AXIS = 4;

    public static int INTAKE_CONTROLLER_ID = 1;

    public static double DEADBAND = 0.05;
  }

  // Constants for the swerve modules
  public final static class SwerveModuleConstants {
    // Module one
    public static boolean SWERVE_MODULE_ONE_ENABLED = true;
    public static int SWERVE_MODULE_ONE_DRIVE = 3;
    public static int SWERVE_MODULE_ONE_TURN = 2;
    public static int SWERVE_MODULE_ONE_ENCODER = 10;
    public static boolean SWERVE_MODULE_ONE_DRIVE_MOTOR_REVERSED = true;
    public static boolean SWERVE_MODULE_ONE_TURNING_MOTOR_REVERSED = true;
    public static double SWERVE_MODULE_ONE_ENCODER_OFFSET_ROT = -0.2571289;
    public static boolean SWERVE_MODULE_ONE_ABSOLUTE_ENCODER_REVERSED = false;

    // Module two
    public static boolean SWERVE_MODULE_TWO_ENABLED = true;
    public static int SWERVE_MODULE_TWO_DRIVE = 5;
    public static int SWERVE_MODULE_TWO_TURN = 4;
    public static int SWERVE_MODULE_TWO_ENCODER = 11;
    public static boolean SWERVE_MODULE_TWO_DRIVE_MOTOR_REVERSED = false;
    public static boolean SWERVE_MODULE_TWO_TURNING_MOTOR_REVERSED = true;
    public static double SWERVE_MODULE_TWO_ENCODER_OFFSET_ROT = 0.051270;
    public static boolean SWERVE_MODULE_TWO_ABSOLUTE_ENCODER_REVERSED = false;

    // Module three
    public static boolean SWERVE_MODULE_THREE_ENABLED = true;
    public static int SWERVE_MODULE_THREE_DRIVE = 7;
    public static int SWERVE_MODULE_THREE_TURN = 6;
    public static int SWERVE_MODULE_THREE_ENCODER = 12;
    public static boolean SWERVE_MODULE_THREE_DRIVE_MOTOR_REVERSED = true;
    public static boolean SWERVE_MODULE_THREE_TURNING_MOTOR_REVERSED = true;
    public static double SWERVE_MODULE_THREE_ENCODER_OFFSET_ROT = -0.496582;
    public static boolean SWERVE_MODULE_THREE_ABSOLUTE_ENCODER_REVERSED = false;

    // Module four
    public static boolean SWERVE_MODULE_FOUR_ENABLED = true;
    public static int SWERVE_MODULE_FOUR_DRIVE = 9;
    public static int SWERVE_MODULE_FOUR_TURN = 8;
    public static int SWERVE_MODULE_FOUR_ENCODER = 13;
    public static boolean SWERVE_MODULE_FOUR_DRIVE_MOTOR_REVERSED = false;
    public static boolean SWERVE_MODULE_FOUR_TURNING_MOTOR_REVERSED = true;
    public static double SWERVE_MODULE_FOUR_ENCODER_OFFSET_ROT = -0.147949;
    public static boolean SWERVE_MODULE_FOUR_ABSOLUTE_ENCODER_REVERSED = false;

    // Extra fields
    public static String SWERVE_CAN_BUS = "swerve";
    public static int GRYO_ID = 14;

    public static double SENSOR_COEFFICENT = 2 * Math.PI;
  }

  // Constants for the movement and kinematics of the swerve system
  public final static class SwerveKinematics {
    public static SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
        new Translation2d(PhysicalConstants.WHEEL_BASE / 2, -PhysicalConstants.TRACK_WIDTH / 2),
        new Translation2d(PhysicalConstants.WHEEL_BASE / 2, PhysicalConstants.TRACK_WIDTH / 2),
        new Translation2d(-PhysicalConstants.WHEEL_BASE / 2, -PhysicalConstants.TRACK_WIDTH / 2),
        new Translation2d(-PhysicalConstants.WHEEL_BASE / 2, PhysicalConstants.TRACK_WIDTH / 2));

    public static double MAX_DRIVE_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;
    public static double MAX_TURN_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI / 2;

    public static double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 15;
    public static double MAX_DRIVE_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND;

    public static double PHYSICAL_MAX_ANGULAR_SPEED_METERS_PER_SECOND = 2.5 * 2.5 * Math.PI;
    public static double MAX_DRIVE_ANGULAR_SPEED_RADIANS_PER_SECOND = PHYSICAL_MAX_ANGULAR_SPEED_METERS_PER_SECOND
        / 2;

    public static double FINE_CONTROL_DIVIDER = 4.0;

    public static double KP = 0.4;
    public static double KI = 0.4;
    public static double KD = 0;

    public static double AUTO_PID_XCONTROLLER_KP = 0;
    public static double AUTO_PID_YCONTROLLER_KP = 0;

    public static double AUTO_PID_THETA_CONTROLLER_KP = 0;

    public static TrapezoidProfile.Constraints AUTO_PID_THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
        MAX_DRIVE_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_TURN_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
  }

  // Constants of physical attributes of the robot
  public final static class PhysicalConstants {
    public static double TRACK_WIDTH = Units.inchesToMeters(21.5);
    public static double WHEEL_BASE = Units.inchesToMeters(21.5);

    public static double SWERVE_WHEEL_DIAMETER = Units.inchesToMeters(3.5);
    public static double SWERVE_GEAR_RATIO = 1 / (150.0 / 7);
  }

}
