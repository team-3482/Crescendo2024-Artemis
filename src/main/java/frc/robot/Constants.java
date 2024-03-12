// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.text.DecimalFormat;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Constants used throughout the code.
 * @implNote BACK of bot is shooter/battery/180 deg, use that as reference for directions
 */
public final class Constants {
    /** Tab names in Shuffleboard */
    public static final class ShuffleboardTabConstants {
        public static final String DEFAULT = "Competition";
        public static final String PITTING = "Utilities";
        // public static final String FIELDS = "Field"; // Disabled for competition
    }

    public static final class IntakeConstants {
        public static final int LEFT_MOTOR_ID = 20;
        public static final int RIGHT_MOTOR_ID = 21;
        public static final int TOP_MOTOR_ID = 14;
        public static final int BOTTOM_MOTOR_ID = 13;

        /** Speed at which to run the intake motors */
        public static final double INTAKE_SPEED = 0.25;
        /** PID Proportional to use when moving the intake down */
        public static final double PIVOT_PID_P_DOWN = 0.05;
        /** Constant speed to use to retract the intake from -1.0 to 1.0*/
        public static final double PIVOT_UP_SPEED = 0.2;

        // /** Position for the intake opened in degrees */
        // public static final int MOTOR_TO_PIVOT_RATIO = 9;

        public enum IntakeState{
            INTAKING(0, 7.5),
            /** The hardware stop angle for the intake when it is idle in degrees*/
            IDLE(160, 25);
            /* Angle of the intake in degrees */
            double intakeAngle;
            double tolerance;

            private IntakeState(double intakeAngle, double tolerance) {
                this.intakeAngle = intakeAngle;
                this.tolerance = tolerance;
            }
            public double getAngle() {
                return this.intakeAngle;
            }
            public double getTolerance() {
                return this.tolerance;
            }
        }
    }

    /** Constants used for the sterilizer */
    public static final class SterilizerConstants {
        public static final int NEO_MOTOR_ID = 10;
        public static final int LASER_ID = 35;
        /** How fast the motor should spin to safely move the note. Between 0 and 1.0 */
        public static final double FEEDING_SPEED = 0.5;
        /** The laser value when a note is at the furthest point from the laser in the sterilizer in millimeters */
        public static final double NOTE_DISTANCE_LASER = 150;
        /** The multiplier for centering the note */
        public static final double NOTE_CALIBRATION_MUL = 0.1;
    }

    /**
     * Constants relating to Shooter code.
     * @implNote LEFT/RIGHT for motors is based on the Note's POV as it travels through the shooter
     */
    public static final class ShooterConstants {
        public static final int LEFT_PIVOT_MOTOR_ID = 20;
        public static final int RIGHT_PIVOT_MOTOR_ID = 21;
        public static final int LEFT_SHOOTER_MOTOR_ID = 11;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 12;
        public static final int HEX_PIVOT_ENCODER_ID = 30;
        
        // Pivot Stuff
        public static final double MOTOR_TO_PIVOT_RATIO = (double) 640 / 3; // 213.33
        /** Lower [0] and upper [1] limits in degrees for the pivot (software stop) */
        public static final double[] PIVOT_ANGLE_LIMITS = new double[]{35, 65};
        /** Allowed pivot error for the pivot rotation in degrees */
        public static final double ALLOWED_PIVOT_ERROR = 0.50;

        // Motion Magic
        public static final class SLOT_0_CONFIGS {
            /** Volts added to overcome friction */
            public static final double kS = 0.12;
            /** Volts added for a target velocity */
            public static final double kV = 0.06; // Target velocity of 100 rps at 0.12 kV
            public static final double kP = 1.2;
            public static final double kI = 0;
            public static final double kD = 0.1;
        }

        /** Cruise velocity in rps */
        public static final int CRUISE_SPEED = 80;
        /** Acceleration in rps/s */
        public static final int CRUISE_ACCELERATION = 80;
        /** Jerk in rps/s^2 (0.1 seconds) */
        public static final int MOTION_MAGIC_JERK = 1600;

        /** Stores all shooter configuration related data */
        public enum ShooterState{
            INTAKE(false, ShooterConstants.PIVOT_ANGLE_LIMITS[0], null, null, null),
            AMP(false, 65.0, 0.18, 675.0, 25.0),
            SPEAKER(false, 65.0, 0.6, 2100.0, 50.0),
            SPEAKER_CALCULATE(true, null, 0.4, 1800.0, 100.0),
            MANUAL(false, null, SPEAKER_CALCULATE.getSpeeds(false)[1], SPEAKER_CALCULATE.getRPMs(false)[1], 100.0)
            ;

            boolean calculateAngle;
            Double positionAngle;
            Double highSpeed;
            Double highRPM;
            Double allowedError;

            private ShooterState(boolean calculateAngle, Double angle, Double highSpeed, Double highRPM, Double allowedError) {
                this.calculateAngle = calculateAngle;
                this.positionAngle = angle;
                this.highSpeed = highSpeed;
                this.highRPM = highRPM;
                this.allowedError = allowedError;
            }
            public boolean calculateAngle() {
                return this.calculateAngle;
            }
            public double getAngle() {
                return this.positionAngle;
            }
            public double[] getSpeeds(boolean invert) {
                return new double[]{
                    this.highSpeed * (this.calculateAngle || invert ? 1 : (double) 2 / 3),
                    this.highSpeed * (!this.calculateAngle || invert ? (double) 2 / 3 : 1)
                };
            }
            public double[] getRPMs(boolean invert) {
                return new double[]{
                    this.highRPM * (this.calculateAngle || invert ? 1 : (double) 2 / 3),
                    this.highRPM * (!this.calculateAngle || invert ? (double) 2 / 3 : 1)
                };
            }
            public double getAllowedError() {
                return this.allowedError;
            }
        }
    }

    /** Values used for running autonomous code */
    public static final class AutonConstants {
        // These are used for on-the-fly paths
        public static final double MAX_LINEAR_VELOCITY = SwerveKinematics.DRIVE_SPEED_COEFFICENT;
        public static final double MAX_LINEAR_ACCELERATION = SwerveKinematics.DRIVE_SLEW_RATE_LIMIT;
        public static final double MAX_ANGULAR_VELOCITY = SwerveKinematics.TURNING_SPEED_COEFFIECENT;
        public static final double MAX_ANGULAR_ACCELERATION = SwerveKinematics.TURNING_SLEW_RATE_LIMIT / 3;

        public static enum PathfindingPosition {
            SPEAKER,
            AMP,
        }

        /** Position the robot will line up to in front of each AprilTag, blue-alliance relative */
        public static final Map<DriverStation.Alliance, Map<PathfindingPosition, Pose2d>> IDEAL_TAG_POSITIONS = Map.ofEntries(
            Map.entry(DriverStation.Alliance.Blue, Map.ofEntries(
                Map.entry(PathfindingPosition.SPEAKER, new Pose2d(new Translation2d(1.34, 5.55), Rotation2d.fromDegrees(180))),
                Map.entry(PathfindingPosition.AMP, new Pose2d(new Translation2d(1.8, 7.66), Rotation2d.fromDegrees(90))))),
            Map.entry(DriverStation.Alliance.Red, Map.ofEntries(
                Map.entry(PathfindingPosition.AMP, new Pose2d(new Translation2d(14.7, 7.66), Rotation2d.fromDegrees(-90))),
                Map.entry(PathfindingPosition.SPEAKER, new Pose2d(new Translation2d(15.2, 5.55), Rotation2d.fromDegrees(180)))))
        );

        /** Initial bot positions used for initializing odometry, blue-alliance relative */
        public static final Map<DriverStation.Alliance, Map<Integer, Pose2d>> STARTING_POSITIONS = Map.ofEntries(
            Map.entry(DriverStation.Alliance.Blue, Map.ofEntries(
                Map.entry(3, new Pose2d(new Translation2d(0.69, 6.69), Rotation2d.fromDegrees(-120))),
                Map.entry(2, new Pose2d(new Translation2d(1.34, 5.55), Rotation2d.fromDegrees(180))),
                Map.entry(1, new Pose2d(new Translation2d(0.69, 4.40), Rotation2d.fromDegrees(120))))),
            Map.entry(DriverStation.Alliance.Red, Map.ofEntries(
                Map.entry(3, new Pose2d(new Translation2d(15.85, 6.69), Rotation2d.fromDegrees(120))),
                Map.entry(2, new Pose2d(new Translation2d(15.2, 5.55), Rotation2d.fromDegrees(180))),
                Map.entry(1, new Pose2d(new Translation2d(15.85, 4.40), Rotation2d.fromDegrees(-120)))))
        );

    }

    /** Constants used by the Swerve Orbit Command */
    public static final class OrbitConstants {
        /** Multipies by the chasis speeds to slow down the bot for more control when orbitting */
        public static final double ORBIT_SPEED_COEFFIECENT = 0.25;
        /**
         * Multipies by the chasis speeds to slow down the bot for more control when
         * orbitting and using fine control
         */
        public static final double ORBIT_FINE_CONTROL_SPEED_COEFFIECENT = 0.125;

        /** The rate of change limit (units per second) for turning limiter in orbit mode */
        public static final double ORBIT_TURNING_SLEW_RATE_LIMIT = SwerveKinematics.TURNING_SLEW_RATE_LIMIT;
        /** The rate limit in units per second for driving in orbit mode (x and y) */
        public static final double ORBIT_DRIVE_SLEW_RATE_LIMIT = SwerveKinematics.DRIVE_SLEW_RATE_LIMIT;

        /** PID constants for controlling the turning speed during orbits */
        public static final class TURNING_SPEED_PID_CONTROLLER {
            /** Tolerance for the PID controller in degrees */
            public static final double TOLERANCE = 0.5;
            public static final double KP = 0.1;
            public static final double KI = 0;
            public static final double KD = 0;
        }

        /** Position in space to orbit (SPEAKER) */
        public static final Map<DriverStation.Alliance, Translation3d> ORBIT_POINT = Map.ofEntries(
            Map.entry(DriverStation.Alliance.Red,
                new Translation3d(16.5, 5.55, 2.47)),
            Map.entry(DriverStation.Alliance.Blue,
                new Translation3d(0, 5.55, 2.47))
        );

        /** The AprilTags to center on at each SPEAKER */
        public static final Map<DriverStation.Alliance, Integer> ORBIT_TAG = Map.ofEntries(
            Map.entry(DriverStation.Alliance.Red, 4),
            Map.entry(DriverStation.Alliance.Blue, 7)
        );
    }

    /** Constants for autos that use the intake limelight */
    public static final class NoteConstants {
        /** The rate of change limit (units per second) for turning limiter in orbit mode */
        public static final double NOTE_TURNING_SLEW_RATE_LIMIT = SwerveKinematics.TURNING_SLEW_RATE_LIMIT;
        /** The rate limit in units per second for driving in orbit mode (x and y) */
        public static final double NOTE_DRIVE_SLEW_RATE_LIMIT = SwerveKinematics.DRIVE_SLEW_RATE_LIMIT;
        /** The input speed the bot should have when driving to a note (between 0 and 1) */
        public static final double NOTE_DRIVE_INPUT_SPEED = 0.25;

        /** Time limit for the centering command in seconds */
        public static final double CENTERING_TIMEOUT = 1.5;

        /** PID constants for controlling the turning speed during centering */
        public static final class TURNING_SPEED_PID_CONTROLLER {
            /** Tolerance for the PID controller in degrees */
            public static final double TOLERANCE = 4;
            public static final double KP = 0.65;
            public static final double KI = 0;
            public static final double KD = 0;
        };
    }

    /** Constants for limelight-related data */
    public static final class LimelightConstants {
        /** Spams "Bad LL 2D/3D Pose Data" when no data is coming from the NetworkTableInstance for LL */
        public static final boolean SPAM_BAD_DATA = false;
        /** Name of the front-facing limelight (shooter / AprilTags) */
        public static final String SHOOTER_LLIGHT = "limelight-three";
        /** Name of the back-facing limelight (Intake / Note Detection) */
        public static final String INTAKE_LLIGHT = "limelight-two";

        /**
         * Only accept limelight values that differ by these x and y values in meters at
         * most from the internal odometer
         */
        public static final double[] ODOMETRY_ALLOWED_ERROR_METERS = new double[] {1, 1};
    }

    /** Constants for the kinematics and driving of the swerve system */
    public static final class SwerveKinematics {
        /** Distance between wheel positions */
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(PhysicalConstants.WHEEL_BASE / 2, -PhysicalConstants.TRACK_WIDTH / 2),
                new Translation2d(PhysicalConstants.WHEEL_BASE / 2, PhysicalConstants.TRACK_WIDTH / 2),
                new Translation2d(-PhysicalConstants.WHEEL_BASE / 2, -PhysicalConstants.TRACK_WIDTH / 2),
                new Translation2d(-PhysicalConstants.WHEEL_BASE / 2, PhysicalConstants.TRACK_WIDTH / 2));

        /** Multiplied by the value given by the slew rate limiter for turning */
        public static final double TURNING_SPEED_COEFFIECENT = Math.PI;
        /** The rate of change limit (units per second) for turning limiter */
        public static final double TURNING_SLEW_RATE_LIMIT = Math.PI;

        /**
         * The max speed a module can reach while driving ; essentially how fast it can
         * rotate while driving full speed, in meters per second.
         * <p> This value should be greater than {@code DRIVE_SPEED_COEFFICIENT}. </p>
         */
        public static final double PHYSICAL_MAX_MODULE_SPEED = 5;
        /**
         * Multiplied by the value given by the slew rate limiter for driving ;
         * basically the top speed reachable.
         * <p> This value should be lower than {@code PHYSICAL_MAX_MODULE_SPEED}. </p>
         */
        public static final double DRIVE_SPEED_COEFFICENT = 4;
        /** The rate of change limit in units per second for driving limiters (x and y) */
        public static final double DRIVE_SLEW_RATE_LIMIT = 1;

        /** Multiplies joystick and turning input by the specified coefficient */
        public static final double FINE_CONTROL_COEFFICENT = 0.25;

        /** x and y speed set by the directional pad in meters per second */
        public static final double D_PAD_SPEED = 0.25; // Previously 1

        /** PID constants used for controlling the turning position of the swerve modules */
        public static final class TURNING_PID_CONTROLLER {
            public static final double KP = 0.325;
            public static final double KI = 0;
            public static final double KD = 0;
        }
    }

    /** Constants of physical attributes of the robot */
    public static final class PhysicalConstants {
        /** Y (not sure if left-right or front-back) distance between wheels in meters */
        public static final double TRACK_WIDTH = Units.inchesToMeters(21.5);
        /** X (not sure if left-right or front-back) distance between wheels in meters */
        public static final double WHEEL_BASE = Units.inchesToMeters(21.5);

        /** Diameter of the wheels in meters */
        public static final double SWERVE_WHEEL_DIAMETER = Units.inchesToMeters(3.5);
        /** Ratio between motor rotations and wheel rotations */
        public static final double SWERVE_MOTOR_TO_WHEEL_RATIO = Math.PI * 5.80 * 2 / 3;

        /** How much to add to the gyro's heading when retrieving it in degrees */
        public static final double GYRO_OFFSET = 180;

        /** Height of the pivot shaft above the floor */
        public static final double SHOOTER_PIVOT_HEIGHT = 0;

        /** What to round decimal values to on Shuffleboard */
        public static final DecimalFormat DEC_FORMAT = new DecimalFormat("#.##");
    }

    /** Constants for the controller and any controller related assignments */
    public static final class ControllerConstants {
        /** DriverStation ID of the driver controller */
        public static final int DRIVE_CONTROLLER_ID = 0;
        /** DriverStation ID of the operator controller */
        public static final int OPERATOR_CONTROLLER_ID = 1;
        /** Removes input around the joystick's center (eliminates stick drift) */
        public static final double DEADBAND = 0.075;
        /** Whether or not to accept directional pad input for movement */
        public static final boolean DPAD_DRIVE_INPUT = true;
    }

    /** Constants used with the LEDSubsystem */
    public static final class LEDConstants {
        /** Port that the LED strip is plugged into */
        public static final int UNDERGLOW_LED_PORT = 7;
        /** Port that the LED strip is plugged into */
        public static final int LEFT_ELEVATOR_LED_PORT = 4;
        /** Port that the LED strip is plugged into */
        public static final int RIGHT_ELEVATOR_LED_PORT = 9;
        /** Number of LEDs to iterate through */
        public static final int UNDERGLOW_LED_COUNT = 150;
        /** Number of LEDs to iterate through */
        public static final int ELEVATOR_LED_COUNT = 18;
    }

    /** Constants for the swerve modules */
    public static final class SwerveModuleConstants {
        /** Configuration for swerve module one */
        public static final class One {
            public static final boolean ENABLED = true;
            public static final int DRIVE = 3;
            public static final int TURN = 2;
            public static final int ENCODER = 10;
            public static final boolean DRIVE_MOTOR_REVERSED = true;
            public static final boolean TURNING_MOTOR_REVERSED = true;
            public static final boolean ABSOLUTE_ENCODER_REVERSED = false;
        }

        /** Configuration for swerve module three */
        public static final class Two {
            public static final boolean ENABLED = true;
            public static final int DRIVE = 5;
            public static final int TURN = 4;
            public static final int ENCODER = 11;
            public static final boolean DRIVE_MOTOR_REVERSED = false;
            public static final boolean TURNING_MOTOR_REVERSED = true;
            public static final boolean ABSOLUTE_ENCODER_REVERSED = false;
        }

        /** Configuration for swerve module three */
        public static final class Three {
            public static final boolean ENABLED = true;
            public static final int DRIVE = 7;
            public static final int TURN = 6;
            public static final int ENCODER = 12;
            public static final boolean DRIVE_MOTOR_REVERSED = true;
            public static final boolean TURNING_MOTOR_REVERSED = true;
            public static final boolean ABSOLUTE_ENCODER_REVERSED = false;
        }

        /** Configuration for swerve module four */
        public static final class Four {
            public static final boolean ENABLED = true;
            public static final int DRIVE = 9;
            public static final int TURN = 8;
            public static final int ENCODER = 13;
            public static final boolean DRIVE_MOTOR_REVERSED = true;
            public static final boolean TURNING_MOTOR_REVERSED = true;
            public static final boolean ABSOLUTE_ENCODER_REVERSED = false;
        }

        /** Name of the CAN bus the swerve is connected to */
        public static final String SWERVE_CAN_BUS = "swerve";
        /** ID of the Pigeon2's connection */
        public static final int GRYO_ID = 14;
    }
}
