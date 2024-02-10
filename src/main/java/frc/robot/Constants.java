// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public final class Constants {
    // Names of the tabs in ShuffleBoard
    public static final class ShuffleboardTabConstants {
        public static String DEFAULT = "Default";
        public static String FIELDS = "Field";
    }
    // Constants used to run autonomous code
    public final static class AutonConstants {
        // These are used for on-the-fly paths
        public static double MAX_DRIVE_SPEED_METERS_PER_SECOND_AUTON = 
            SwerveKinematics.MAX_DRIVE_SPEED_METERS_PER_SECOND;
        public static double MAX_DRIVE_ACCELERATION_METERS_PER_SECOND_SQUARED_AUTON = 
            SwerveKinematics.MAX_DRIVE_ACCELERATION_METERS_PER_SECOND_SQUARED;
        public static double MAX_DRIVE_ANGULAR_SPEED_RADIANS_PER_SECOND_AUTON = 
            SwerveKinematics.MAX_DRIVE_ANGULAR_SPEED_RADIANS_PER_SECOND;
        public static double MAX_TURN_ACCELERATION_RADIANS_PER_SECOND_SQUARED_AUTON = 
            SwerveKinematics.MAX_TURN_ACCELERATION_RADIANS_PER_SECOND_SQUARED;
        // This is the deviation allowed to the robot when orbiting an AprilTag
        // These are where the bot should ideally line up for each AprilTag 
        public static Map<Integer, Pose2d> IDEAL_TAG_POSITIONS = Map.ofEntries(
            // Map.entry(3, new Pose2d(new Translation2d(15.0, 5.6), new Rotation2d())),
            Map.entry(4, new Pose2d(new Translation2d(15.0, 5.4), Rotation2d.fromDegrees(90)))
        );
        
        // Initial bot positions to initialize odometry
        public static Map<DriverStation.Alliance, Map<Integer, Pose2d>> STARTING_POSITIONS = Map.ofEntries( 
            Map.entry(DriverStation.Alliance.Blue, Map.ofEntries(
                Map.entry(1, new Pose2d(new Translation2d(), new Rotation2d())),
                Map.entry(2, new Pose2d(new Translation2d(), new Rotation2d())),
                Map.entry(3, new Pose2d(new Translation2d(), new Rotation2d()))
            )),
            Map.entry(DriverStation.Alliance.Red, Map.ofEntries(
                Map.entry(1, new Pose2d(new Translation2d(16, 4.4), Rotation2d.fromDegrees(55))),
                Map.entry(2, new Pose2d(new Translation2d(15.4, 5.4), Rotation2d.fromDegrees(0))),
                Map.entry(3, new Pose2d(new Translation2d(16, 7), Rotation2d.fromDegrees(-60)))
            ))
        );
        
    }
    public final static class OrbitConstants {
        public static double ORBIT_FINER_MOVEMENT = 4;
        public static double ORBIT_FINEST_MOVEMENT = 8;

        public static double MAX_DRIVE_ACCELERATION_METERS_PER_SECOND_SQUARED_ORBIT = 4;
        
        public static double KP = 1;
        public static double KI = 0;
        public static double KD = 0;
    }
    
    // Constants for limelight-related data
    public final static class LimelightConstants {
        public static double TIMEOUT = 5; // In seconds
        public static String FRONT_LIMELIGHT = "limelight-three"; // Shooter / AprilTags
        public static String BACK_LIMELIGHT = "limelight-two"; // Intake / Detection

        public static double[] ODOMETRY_ALLOWED_ERROR_METERS = new double[]{1, 1}; // x, y
    }
    // Constants for the controller and any controller related items
    // (ex. buttons, axis, ect.)
    public final static class ControllerConstants {
        public static int DRIVE_CONTROLLER_ID = 0;
        public static int DRIVE_ROT_AXIS = 4;
        public static int INTAKE_CONTROLLER_ID = 1;
        public static double DEADBAND = 0.075;

        // Input
        public static boolean DPAD_DRIVE_INPUT = true;
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
            public static boolean DRIVE_MOTOR_REVERSED = true;
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
        public static SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(PhysicalConstants.WHEEL_BASE / 2, -PhysicalConstants.TRACK_WIDTH / 2),
            new Translation2d(PhysicalConstants.WHEEL_BASE / 2, PhysicalConstants.TRACK_WIDTH / 2),
            new Translation2d(-PhysicalConstants.WHEEL_BASE / 2, -PhysicalConstants.TRACK_WIDTH / 2),
            new Translation2d(-PhysicalConstants.WHEEL_BASE / 2, PhysicalConstants.TRACK_WIDTH / 2));

        public static double MAX_DRIVE_ANGULAR_SPEED_RADIANS_PER_SECOND =  2 * Math.PI;
        public static double MAX_TURN_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 15;
        public static double MAX_DRIVE_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        public static double MAX_DRIVE_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;

        public static double FINE_CONTROL_DIVIDER = 4.0;

        public static double KP = 0.325;
        public static double KI = 0;
        public static double KD = 0;

        public static TrapezoidProfile.Constraints PID_ROTATION_TRAPEZOID_PROFILE =
            new TrapezoidProfile.Constraints(
                MAX_DRIVE_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_TURN_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    }

    // Constants of physical attributes of the robot
    public final static class PhysicalConstants {
        public static double TRACK_WIDTH = Units.inchesToMeters(21.5);
        public static double WHEEL_BASE = Units.inchesToMeters(21.5);

        public static double SWERVE_WHEEL_DIAMETER = Units.inchesToMeters(3.5);
        public static double SWERVE_MOTOR_TO_WHEEL_RATIO = Math.PI * 5.80 * 2 / 3;
        
        public static double DIST_BETWEEN_AMP_TAGS_METERS = 0.43;
    }
  
  public static class LEDConstants {
    public static int ledPort = 0;
    public static int ledCount = 150;

    // colors use 3 ints for RGB
    public static int[] off = { 0, 0, 0 };
    public static int[] redColor = { 255, 0, 0 };
    public static int[] blueColor = { 0, 0, 255 };
  }
}
