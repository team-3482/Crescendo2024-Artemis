// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public final class Constants {
    /** Tab names in Shuffleboard */
    public static final class ShuffleboardTabConstants {
        public static String DEFAULT = "Default";
        public static String FIELDS = "Field";
    }

    /** Values used for running autonomous code */
    public final static class AutonConstants {
        // These are used for on-the-fly paths
        public static double MAX_LINEAR_VELOCITY =  1;
        public static double MAX_LINEAR_ACCELERATION = 1;
        public static double MAX_ANGULAR_VELOCITY = Math.PI / 2;
        public static double MAX_ANGULAR_ACCELERATION = Math.PI / 2;
        
        public static char AMP = 'a';
        public static char SPEAKER = 's';
        /** Position the robot will line up to in front of each AprilTag, blue-alliance relative */
        public static Map<DriverStation.Alliance, Map<Character, Pose2d>> IDEAL_TAG_POSITIONS = Map.ofEntries(
            Map.entry(DriverStation.Alliance.Blue, Map.ofEntries(
                Map.entry(AMP, new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0))),
                Map.entry(SPEAKER, new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)))
            )),
            Map.entry(DriverStation.Alliance.Red, Map.ofEntries(
                Map.entry(AMP, new Pose2d(new Translation2d(15.5, 5.4), Rotation2d.fromDegrees(0))), // Previously 15
                Map.entry(SPEAKER, new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)))
            ))
        );
        
        /** Initial bot positions used for initializing odometry, blue-alliance relative */
        public static Map<DriverStation.Alliance, Map<Integer, Pose2d>> STARTING_POSITIONS = Map.ofEntries( 
            Map.entry(DriverStation.Alliance.Blue, Map.ofEntries(
                Map.entry(1, new Pose2d(new Translation2d(1,1), new Rotation2d())),
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
  
    /** Constants used by the Swerve Orbit Command */
    public final static class OrbitConstants {
        /** Multipies by the chasis speeds to slow down the bot for more control when orbitting */
        public static double ORBIT_SPEED_COEFFIECENT = 0.25; 
        /** Multipies by the chasis speeds to slow down the bot for more control when orbitting and using fine control */
        public static double ORBIT_FINE_CONTROL_SPEED_COEFFIECENT = 0.125; 

        /** The rate of change limit (units per second) for turning limiter in orbit mode */
        public static double ORBIT_TURNING_SLEW_RATE_LIMIT = SwerveKinematics.TURNING_SLEW_RATE_LIMIT; 
        /** The rate limit in units per second for driving in orbit mode (x and y) */
        public static double ORBIT_DRIVE_SLEW_RATE_LIMIT = SwerveKinematics.DRIVE_SLEW_RATE_LIMIT;

        /** PID constants for controlling the turning speed during orbits */
        public static class TURNING_SPEED_PID_CONTROLLER {
            /** Tolerance for the PID controller in degrees */
            public static double TOLERANCE = 1;
            public static double KP = 0.5;
            public static double KI = 0;
            public static double KD = 0;
        }

        /** Tag IDs acceptable for orbit (keys) and the x offset needed for them in meters (values) */
        public static Map<Integer, Double> ORBIT_IDS = Map.ofEntries(
            Map.entry(4, 0.0), // Orbit Tag 4
            Map.entry(3, PhysicalConstants.DIST_AMP_TAGS), // Orbit Tag 4
            Map.entry(7, 0.0), // Orbit Tag 7
            Map.entry(8, -PhysicalConstants.DIST_AMP_TAGS) // Orbit Tag 7
        );
    }
    
    // Constants for limelight-related data
    public final static class LimelightConstants {
        /** Name of the front-facing limelight (shooter / AprilTags) */
        public static String FRONT_LIMELIGHT = "limelight-three";
        /** Name of the backwards-facing limelight (Intake / Note Detection) */
        public static String BACK_LIMELIGHT = "limelight-two";
        
        /** Only accepts limelight values that differ by these x and y values from the internal odometer */
        public static double[] ODOMETRY_ALLOWED_ERROR_METERS = new double[]{1, 1};
    }

    /** Constants for the kinematics and driving of the swerve system */
    public final static class SwerveKinematics {
        /** Distance between wheel positions */
        public static SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(PhysicalConstants.WHEEL_BASE / 2, -PhysicalConstants.TRACK_WIDTH / 2),
            new Translation2d(PhysicalConstants.WHEEL_BASE / 2, PhysicalConstants.TRACK_WIDTH / 2),
            new Translation2d(-PhysicalConstants.WHEEL_BASE / 2, -PhysicalConstants.TRACK_WIDTH / 2),
            new Translation2d(-PhysicalConstants.WHEEL_BASE / 2, PhysicalConstants.TRACK_WIDTH / 2));

        /** Multiplied by the value given by the slew rate limiter for turning */
        public static double TURNING_SPEED_COEFFIECENT = Math.PI; 
        /** The rate of change limit (units per second) for turning limiter */
        public static double TURNING_SLEW_RATE_LIMIT = Math.PI; 

        /** The max speed a module can reach while driving ; essentially how fast it can rotate while driving full speed, in meters per second */
        public static double PHYSICAL_MAX_MODULE_SPEED = 5; // This value should be greater than DRIVE_SPEED_COEFFICENT
        /** Multiplied by the value given by the slew rate limiter for driving ; basically the top speed reachable*/
        public static double DRIVE_SPEED_COEFFICENT = 4; // This value should be lower than PHYSICAL_MAX_MODULE_SPEED
        /** The rate of change limit in units per second for driving limiters (x and y) */
        public static double DRIVE_SLEW_RATE_LIMIT = 1; 

        /** Multiplies joystick and turning input by the specified coefficient */
        public static double FINE_CONTROL_COEFFICENT = 0.25;
        
        /** x and y speed set by the directional pad in meters per second */
        public static double D_PAD_SPEED = 0.25; // Previously 1
        
        /** PID constants used for controlling the turning position of the swerve modules */
        public static class TURNING_PID_CONTROLLER {
            public static double KP = 0.325;
            public static double KI = 0; 
            public static double KD = 0; 
        }
    }

    /** Constants of physical attributes of the robot */
    public final static class PhysicalConstants {
        public static double TRACK_WIDTH = Units.inchesToMeters(21.5); // Y (not sure if left-right or front-back) distance between wheels
        public static double WHEEL_BASE = Units.inchesToMeters(21.5); // X (not sure if left-right or front-back) distance between wheels
        
        /** Diameter in meters */
        public static double SWERVE_WHEEL_DIAMETER = Units.inchesToMeters(3.5); // Diameter of the wheels
        /** Ratio between motor rotations and wheel rotations */
        public static double SWERVE_MOTOR_TO_WHEEL_RATIO = Math.PI * 5.80 * 2 / 3;
        /** Distance between the two april tags on the AMP in meters*/
        public static double DIST_AMP_TAGS = 0.43;
    }
  
    /** Constants for the controller and any controller related assignments */
    public final static class ControllerConstants {
        /** DriverStation ID of the driver controller */
        public static int DRIVE_CONTROLLER_ID = 0;
        /** DriverStation ID of the operator controller */
        public static int OPERATOR_CONTROLLER_ID = 1;
        /** Removes input around the joystick's center (eliminates stick drift) */
        public static double DEADBAND = 0.075;

        /** Whether or not to accept directional pad input for movement */
        public static boolean DPAD_DRIVE_INPUT = true;
    }
  
    /** Constants used with the LEDSubsystem */
    public static class LEDConstants {
        /** Port that the LED strip is plugged into */
        public static int LED_PORT = 0;
        /** Number of LEDs to iterate through */
        public static int LED_COUNT = 150;

        /** RGB values for red */
        public static int[] RED_COLOR = { 255, 0, 0 };
        /** RGB values for blue */
        public static int[] BLUE_COLOR = { 0, 0, 255 };
    }
    
    /** Constants for the swerve modules */
    public final static class SwerveModuleConstants {
        /** Configuration for swerve module one */
        public final static class One {
            public static boolean ENABLED = true;
            public static int DRIVE = 3;
            public static int TURN = 2;
            public static int ENCODER = 10;
            public static boolean DRIVE_MOTOR_REVERSED = true;
            public static boolean TURNING_MOTOR_REVERSED = true;
            public static boolean ABSOLUTE_ENCODER_REVERSED = false;
        }

        /** Configuration for swerve module three */
        public final static class Two {
            public static boolean ENABLED = true;
            public static int DRIVE = 5;
            public static int TURN = 4;
            public static int ENCODER = 11;
            public static boolean DRIVE_MOTOR_REVERSED = false;
            public static boolean TURNING_MOTOR_REVERSED = true;
            public static boolean ABSOLUTE_ENCODER_REVERSED = false;
        }

        /** Configuration for swerve module three */
        public final static class Three {
            public static boolean ENABLED = true;
            public static int DRIVE = 7;
            public static int TURN = 6;
            public static int ENCODER = 12;
            public static boolean DRIVE_MOTOR_REVERSED = true;
            public static boolean TURNING_MOTOR_REVERSED = true;
            public static boolean ABSOLUTE_ENCODER_REVERSED = false;
        }

        /** Configuration for swerve module four */
        public final static class Four {
            public static boolean ENABLED = true;
            public static int DRIVE = 9;
            public static int TURN = 8;
            public static int ENCODER = 13;
            public static boolean DRIVE_MOTOR_REVERSED = true;
            public static boolean TURNING_MOTOR_REVERSED = true;
            public static boolean ABSOLUTE_ENCODER_REVERSED = false;
        }

        /** Name of the CAN bus the swerve is connected to */
        public static String SWERVE_CAN_BUS = "swerve";
        /** ID of the Pigeon2's connection */
        public static int GRYO_ID = 14;
    }
}
