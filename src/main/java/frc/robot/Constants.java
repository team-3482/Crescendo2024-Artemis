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

/** Constants used throughout the code.
 * Remember : BACK of bot is shooter/battery, use that as reference for directions.
 * Remember : BACK of the bot is 0 degrees in heading
 */
public final class Constants {
    /** Tab names in Shuffleboard */
    public static final class ShuffleboardTabConstants {
        public static String DEFAULT = "Default";
        public static String FIELDS = "Field";
    }

    /** Constants used for the sterilizer */
    public static final class SterilizerConstants {
        public static int NEO_MOTOR_ID = 24;
        public static int LASER_ID = 25;
        /** How fast the motor should spin in rpm to safely move the note */
        public static double MOVING_SPEED = 50;
        /** The laser value when a note is at the furthest point from the laser in the sterilizer in millimeters*/
        public static double NOTE_DISTANCE_LASER = 20;
    }

    /**
     * Constants relating to Shooter code.
     * Remember : LEFT/RIGHT for motors is based on the Note's POV as it travels through the shooter
     */
    public static final class ShooterConstants {
        public static int LEFT_SHOOTER_MOTOR_ID = 20;
        public static int RIGHT_SHOOTER_MOTOR_ID = 21;
        public static int LEFT_PIVOT_MOTOR_ID = 22;
        public static int RIGHT_PIVOT_MOTOR_ID = 23;
        public static int HEX_PIVOT_ENCODER_ID = 26;

        // Shooting stuff
        /** Allowed RPM error for the shooter motors */
        public static double ALLOWED_RPM_ERROR = Units.rotationsPerMinuteToRadiansPerSecond(10);
        /** Allowed pivot error for the pivot rotation in degrees */
        public static double ALLOWED_PIVOT_ERROR = 1;

        // Pivot Stuff
        public static int MOTOR_TO_PIVOT_RATIO = 240;
        public static final class SLOT_0_CONFIGS {
            /** Volts added to overcome friction */
            public static double kS = 0.24;
            /** Volts added for a target velocity */
            public static double kV = 0.12; // Target velocity of 100 rps
            public static double kP = 4.8;
            public static double kI = 0;
            public static double kD = 0.1;
        }
        
        /** Cruise velocity in rps */
        public static int CRUISE_SPEED = 80;
        /** Acceleration in rps/s */
        public static int CRUISE_ACCELERATION = 160;
        /** Jerk in rps/s^2 (0.1 seconds)*/
        public static int MOTION_MAGIC_JERK = 1600;
    }

    /** Values used for running autonomous code */
    public static final class AutonConstants {
        // These are used for on-the-fly paths
        public static double MAX_LINEAR_VELOCITY = SwerveKinematics.DRIVE_SPEED_COEFFICENT;
        public static double MAX_LINEAR_ACCELERATION = SwerveKinematics.DRIVE_SLEW_RATE_LIMIT;
        public static double MAX_ANGULAR_VELOCITY = SwerveKinematics.TURNING_SPEED_COEFFIECENT;
        public static double MAX_ANGULAR_ACCELERATION = SwerveKinematics.TURNING_SLEW_RATE_LIMIT / 3;
        
        // This should be an enum
        public static char AMP = 'a';
        public static char SPEAKER = 's';
        /** Position the robot will line up to in front of each AprilTag, blue-alliance relative */
        public static Map<DriverStation.Alliance, Map<Character, Pose2d>> IDEAL_TAG_POSITIONS = Map.ofEntries(
            Map.entry(DriverStation.Alliance.Blue, Map.ofEntries(
                Map.entry(SPEAKER, new Pose2d(new Translation2d(1.34, 5.55), Rotation2d.fromDegrees(180))),
                Map.entry(AMP, new Pose2d(new Translation2d(1.8, 7.66), Rotation2d.fromDegrees(90)))
            )),
            Map.entry(DriverStation.Alliance.Red, Map.ofEntries(
                Map.entry(AMP, new Pose2d(new Translation2d(14.7, 7.66), Rotation2d.fromDegrees(-90))),
                Map.entry(SPEAKER, new Pose2d(new Translation2d(15.2, 5.55), Rotation2d.fromDegrees(180)))
            ))
        );
        
        /** Initial bot positions used for initializing odometry, blue-alliance relative */
        public static Map<DriverStation.Alliance, Map<Integer, Pose2d>> STARTING_POSITIONS = Map.ofEntries( 
            Map.entry(DriverStation.Alliance.Blue, Map.ofEntries(
                Map.entry(3, new Pose2d(new Translation2d(0.69, 6.69), Rotation2d.fromDegrees(-120))),
                Map.entry(2, new Pose2d(new Translation2d(1.34, 5.55), Rotation2d.fromDegrees(0))),
                Map.entry(1, new Pose2d(new Translation2d(0.69, 4.40), Rotation2d.fromDegrees(120)))
            )),
            Map.entry(DriverStation.Alliance.Red, Map.ofEntries(
                Map.entry(3, new Pose2d(new Translation2d(15.85, 6.69), Rotation2d.fromDegrees(120))),
                Map.entry(2, new Pose2d(new Translation2d(15.2, 5.55), Rotation2d.fromDegrees(0))),
                Map.entry(1, new Pose2d(new Translation2d(15.85, 4.40), Rotation2d.fromDegrees(-120)))
            ))
        );
        
    }
  
    /** Constants used by the Swerve Orbit Command */
    public static final class OrbitConstants {
        /** Multipies by the chasis speeds to slow down the bot for more control when orbitting */
        public static double ORBIT_SPEED_COEFFIECENT = 0.25;
        /** Multipies by the chasis speeds to slow down the bot for more control when orbitting and using fine control */
        public static double ORBIT_FINE_CONTROL_SPEED_COEFFIECENT = 0.125; 

        /** The rate of change limit (units per second) for turning limiter in orbit mode */
        public static double ORBIT_TURNING_SLEW_RATE_LIMIT = SwerveKinematics.TURNING_SLEW_RATE_LIMIT; 
        /** The rate limit in units per second for driving in orbit mode (x and y) */
        public static double ORBIT_DRIVE_SLEW_RATE_LIMIT = SwerveKinematics.DRIVE_SLEW_RATE_LIMIT;

        /** PID constants for controlling the turning speed during orbits */
        public static final class TURNING_SPEED_PID_CONTROLLER {
            /** Tolerance for the PID controller in degrees */
            public static double TOLERANCE = 0.5;
            public static double KP = 0.55;
            public static double KI = 0;
            public static double KD = 0;
        }

        /** Position in space to orbit (SPEAKER) */
        public static Map<DriverStation.Alliance, Translation2d> ORBIT_POINT = Map.ofEntries( 
            Map.entry(DriverStation.Alliance.Red,
                new Translation2d(16.5, 5.55)
            ),
            Map.entry(DriverStation.Alliance.Blue,
                new Translation2d(0, 5.55)
            )
        );
    }

    /** Constants for autos that use the intake limelight */
    public static final class NoteConstants {
        /** The rate of change limit (units per second) for turning limiter in orbit mode */
        public static double NOTE_TURNING_SLEW_RATE_LIMIT = SwerveKinematics.TURNING_SLEW_RATE_LIMIT; 
        /** The rate limit in units per second for driving in orbit mode (x and y) */
        public static double NOTE_DRIVE_SLEW_RATE_LIMIT = SwerveKinematics.DRIVE_SLEW_RATE_LIMIT;
        /** The input speed the bot should have when driving to a note (between 0 and 1) */
        public static double NOTE_DRIVE_INPUT_SPEED = 0.25;

        /** Time limit for the centering command in seconds */
        public static double CENTERING_TIMEOUT = 1.5;
        /** PID constants for controlling the turning speed during centering */
        public static final class TURNING_SPEED_PID_CONTROLLER {
            /** Tolerance for the PID controller in degrees */
            public static double TOLERANCE = 2;
            public static double KP = 0.65;
            public static double KI = 0;
            public static double KD = 0;
        };
    }
    
    /** Constants for limelight-related data */
    public static final class LimelightConstants {
        /** Name of the front-facing limelight (shooter / AprilTags) */
        public static String SHOOTER_LLIGHT = "limelight-three";
        /** Name of the back-facing limelight (Intake / Note Detection) */
        public static String INTAKE_LLIGHT = "limelight-two";
        
        /** Only accept limelight values that differ by these x and y values in meters at most from the internal odometer */
        public static double[] ODOMETRY_ALLOWED_ERROR_METERS = new double[]{1, 1};
    }

    /** Constants for the kinematics and driving of the swerve system */
    public static final class SwerveKinematics {
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
        public static final class TURNING_PID_CONTROLLER {
            public static double KP = 0.325;
            public static double KI = 0; 
            public static double KD = 0; 
        }
    }

    /** Constants of physical attributes of the robot */
    public static final class PhysicalConstants {
        /** Y (not sure if left-right or front-back) distance between wheels in meters */
        public static double TRACK_WIDTH = Units.inchesToMeters(21.5); 
        /** X (not sure if left-right or front-back) distance between wheels in meters */
        public static double WHEEL_BASE = Units.inchesToMeters(21.5);
        
        /** Diameter of the wheels in meters */
        public static double SWERVE_WHEEL_DIAMETER = Units.inchesToMeters(3.5);
        /** Ratio between motor rotations and wheel rotations */
        public static double SWERVE_MOTOR_TO_WHEEL_RATIO = Math.PI * 5.80 * 2 / 3;

        /** How much to add to the gyro's heading when retrieving it in degrees */
        public static double GYRO_OFFSET = 180;
    }
  
    /** Constants for the controller and any controller related assignments */
    public static final class ControllerConstants {
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
    public static final class LEDConstants {
        /** Port that the LED strip is plugged into */
        public static int UNDERGLOW_LED_PORT = 9;
        /** Number of LEDs to iterate through */
        public static int UNDERGLOW_LED_COUNT = 150;
    }
    
    /** Constants for the swerve modules */
    public static final class SwerveModuleConstants {
        /** Configuration for swerve module one */
        public static final class One {
            public static boolean ENABLED = true;
            public static int DRIVE = 3;
            public static int TURN = 2;
            public static int ENCODER = 10;
            public static boolean DRIVE_MOTOR_REVERSED = true;
            public static boolean TURNING_MOTOR_REVERSED = true;
            public static boolean ABSOLUTE_ENCODER_REVERSED = false;
        }

        /** Configuration for swerve module three */
        public static final class Two {
            public static boolean ENABLED = true;
            public static int DRIVE = 5;
            public static int TURN = 4;
            public static int ENCODER = 11;
            public static boolean DRIVE_MOTOR_REVERSED = false;
            public static boolean TURNING_MOTOR_REVERSED = true;
            public static boolean ABSOLUTE_ENCODER_REVERSED = false;
        }

        /** Configuration for swerve module three */
        public static final class Three {
            public static boolean ENABLED = true;
            public static int DRIVE = 7;
            public static int TURN = 6;
            public static int ENCODER = 12;
            public static boolean DRIVE_MOTOR_REVERSED = true;
            public static boolean TURNING_MOTOR_REVERSED = true;
            public static boolean ABSOLUTE_ENCODER_REVERSED = false;
        }

        /** Configuration for swerve module four */
        public static final class Four {
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
  public static class ShooterConstants {
    public static final int TOP_MOTOR_ID = 12;
    public static final int BOTTOM_MOTOR_ID = 11;
    public static final int FEEDER_MOTOR_ID = 13;
    public static final double SHOOTER_SPEED = 0.66;
    public static final double FEEDER_SPEED = 0.20;
  }

  public static class IntakeConstants {
    public static final int LEFT_MOTOR_ID = 15;
    public static final int RIGHT_MOTOR_ID = 14;
    public static final int INTAKE_MOTOR_ID = 16;

    public static final double INTAKE_SPEED = 0.25;
    public static final double PIVOT_SPEED = 0.02;
    public static final double PIVOT_TOLERANCE = Units.degreesToRadians(2.5) * 9;

    public static final int PIVOT_DOWN_DEGREE = -90 * 9;
    public static final int PIVOT_UP_DEGREE = 0 * 9;
  }

}
