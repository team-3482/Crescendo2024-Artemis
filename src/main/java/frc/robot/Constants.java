// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    // Constants used to run Auton code
    public final static class AutonConstants {
        public static String AUTON_PATH_NAME = "StraightLine1Meter";
    }
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
        // Module One
        public final static class One {
            public static boolean ENABLED = true;
            public static int DRIVE = 3;
            public static int TURN = 2;
            public static int ENCODER = 10;
            public static boolean DRIVE_MOTOR_REVERSED = true;
            public static boolean TURNING_MOTOR_REVERSED = true;
            public static double ENCODER_OFFSET_ROT = 0;
            public static boolean ABSOLUTE_ENCODER_REVERSED = false;
        }

        // Module two
        public final static class Two {
            public static boolean ENABLED = true;
            public static int DRIVE = 5;
            public static int TURN = 4;
            public static int ENCODER = 11;
            public static boolean DRIVE_MOTOR_REVERSED = false;
            public static boolean TURNING_MOTOR_REVERSED = true;
            public static double ENCODER_OFFSET_ROT = 0;
            public static boolean ABSOLUTE_ENCODER_REVERSED = false;
        }

        // Module three
        public final static class Three {
            public static boolean ENABLED = true;
            public static int DRIVE = 7;
            public static int TURN = 6;
            public static int ENCODER = 12;
            public static boolean DRIVE_MOTOR_REVERSED = true;
            public static boolean TURNING_MOTOR_REVERSED = true;
            public static double ENCODER_OFFSET_ROT = 0;
            public static boolean ABSOLUTE_ENCODER_REVERSED = false;
        }

        // Module four
        public final static class Four {
            public static boolean ENABLED = true;
            public static int DRIVE = 9;
            public static int TURN = 8;
            public static int ENCODER = 13;
            public static boolean DRIVE_MOTOR_REVERSED = false;
            public static boolean TURNING_MOTOR_REVERSED = true;
            public static double ENCODER_OFFSET_ROT = 0;
            public static boolean ABSOLUTE_ENCODER_REVERSED = false;
        }

        // Extra fields
        public static String SWERVE_CAN_BUS = "swerve";
        public static int GRYO_ID = 14;
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
        public static double MAX_DRIVE_ANGULAR_SPEED_RADIANS_PER_SECOND = PHYSICAL_MAX_ANGULAR_SPEED_METERS_PER_SECOND / 2;

        public static double FINE_CONTROL_DIVIDER = 4.0;

        public static double KP = 0.4;
        public static double KI = 0.4;
        public static double KD = 0;

        public static double AUTO_PID_XCONTROLLER_KP = 0;
        public static double AUTO_PID_YCONTROLLER_KP = 0;

        public static double AUTO_PID_THETA_CONTROLLER_KP = 0;

        public static TrapezoidProfile.Constraints AUTO_PID_THETA_CONTROLLER_CONSTRAINTS =
            new TrapezoidProfile.Constraints(
                MAX_DRIVE_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_TURN_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    }

    // Constants of physical attributes of the robot
    public final static class PhysicalConstants {
        public static double TRACK_WIDTH = Units.inchesToMeters(21.5);
        public static double WHEEL_BASE = Units.inchesToMeters(21.5);

        public static double SWERVE_WHEEL_DIAMETER = Units.inchesToMeters(3.5);
        public static double SWERVE_GEAR_RATIO = 1 / (150.0 / 7);
        public static double ROT_TO_RAD = 2 * Math.PI;
    }
}
