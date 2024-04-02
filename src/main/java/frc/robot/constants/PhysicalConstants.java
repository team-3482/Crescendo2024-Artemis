// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * Constants used throughout the code specifically related to subsystems or unchangeable aspects of the bot.
 * @implNote BACK of bot is shooter/battery/180 deg, use that as reference for directions
 */
public final class PhysicalConstants {
    /** Constants of physical attributes of the robot */
    public static final class RobotConstants {
        /** Height of the pivot shaft above the floor */
        public static final double SHOOTER_PIVOT_HEIGHT = 0.45;
        /** Y (not sure if left-right or front-back) distance between wheels in meters */
        public static final double TRACK_WIDTH = Units.inchesToMeters(21.5);
        /** X (not sure if left-right or front-back) distance between wheels in meters */
        public static final double WHEEL_BASE = Units.inchesToMeters(21.5);

        /** Diameter of the wheels in meters */
        public static final double SWERVE_WHEEL_DIAMETER = Units.inchesToMeters(3.5);
        /** Ratio between motor rotations and wheel rotations. This is used for odometry. */
        public static final double SWERVE_MOTOR_TO_WHEEL_RATIO = Math.PI * 5.80 * 2 / 3;
        
        /** The loop time in seconds for publishing telemetry */
        public static final double TELEMETRY_LOOP_TIME = 0.25;
        
        /** Name of the CAN bus the swerve is connected to */
        public static final String SWERVE_CAN_BUS = "swerve";

        /** CAN ID for the Pigeon2 */
        public static final int GYRO_ID = 14;
        /** How much to add to the gyro's heading when retrieving it in degrees */
        public static final double GYRO_OFFSET = 180;
    }

    public static final class IntakeConstants {
        public static final int LEFT_MOTOR_ID = 20;
        public static final int RIGHT_MOTOR_ID = 21;
        public static final int TOP_MOTOR_ID = 14;
        public static final int BOTTOM_MOTOR_ID = 13;

        /** Speed at which to run the intake motors */
        public static final double INTAKE_SPEED = 0.25;
        /** PID Proportional to use when moving the intake up */
        public static final double PIVOT_PID_P_UP = 0.11;
        /** PID Proportional to use when moving the intake down */
        public static final double PIVOT_PID_P_DOWN = 0.075;
        /** Constant speed to use to retract the intake from 0 to 1.0*/
        public static final double PIVOT_UP_SPEED = 0.2;
        /** Constant speed to use to lower the intake from -1.0 to 0*/
        public static final double PIVOT_DOWN_SPEED = 0.1;
    }

    /** Constants used for the sterilizer */
    public static final class SterilizerConstants {
        public static final int NEO_MOTOR_ID = 10;
        public static final int BACK_LASER_ID = 35;
        public static final int FRONT_LASER_ID = 34;
        /** How fast the motor should spin to move the note. Between 0 and 1.0 */
        public static final double FEEDING_SPEED = 0.65;
        /** How fast the motor should spin to adjust the note. Between 0 and 1.0 */
        public static final double ADJUSTING_SPEED = 0.1;
        /** The laser value when a note is at the furthest point from the laser in the sterilizer in millimeters */
        public static final double NOTE_DISTANCE_LASER = 200;
    }

    /**
     * Constants relating to Shooter code.
     * @implNote LEFT/RIGHT for motors is based on the Note's POV as it travels through the shooter
     */
    public static final class ShooterConstants {
        // Shooting wheels
        public static final int LEFT_SHOOTER_MOTOR_ID = 11;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 12;
        
        // Pivot
        public static final int LEFT_PIVOT_MOTOR_ID = 20;
        public static final int RIGHT_PIVOT_MOTOR_ID = 21;
        public static final int LEFT_CANCODER_ID = 22;
        public static final int RIGHT_CANCODER_ID = 23;
        
        /** Motor configurations for the shooting wheels */
        public static final class Shooting {
            /** P of PID for the Shooter wheeels */
            public static final double kP_SHOOTING = 0.001;
            /** Feed forward for the Shooter wheels */
            public static final double kFF_SHOOTING = 0.000265;
        }

        // Pivot Stuff
        /** Motion magic configurations for pivot motors */
        public static final class slot0Configs {
            /** Volts added to overcome friction */
            public static final double kS = 0.12;
            /** Volts added for a target velocity */
            public static final double kV = 0.06; // Target velocity of 100 rps at 0.12 kV
            public static final double kP = 1.2;
            public static final double kI = 0;
            public static final double kD = 0.1;
        }

        /** Motor onfigurations for moving the pivot */
        public static final class Pivot {
            // public static final double MOTOR_TO_PIVOT_RATIO = (double) 640 / 3; // 213.33
            /** Lower [0] and upper [1] limits in degrees for the pivot (software stop) */
            public static final double[] ANGLE_LIMITS = new double[]{35, 63};
            /** Allowed pivot error for the pivot rotation in degrees */
            public static final double ALLOWED_ERROR = 0.5;
            /** P of PPID for the pivot motors */
            public static final double kP_PIVOT = 0.08;
            // Constraints for TrapezoidProfile for the pivot motors
            public static final double MAX_VEL = 60;
            public static final double MAX_ACCEL = 40;
        }

        /** Cruise velocity in rps */
        public static final double CRUISE_SPEED = 80;
        /** Acceleration in rps/s */
        public static final double CRUISE_ACCELERATION = 80;
        /** Jerk in rps/s^2 (0.1 seconds) */
        public static final double MOTION_MAGIC_JERK = 1600;
    }

    /** Constants for Elevator motors */ 
    public static final class ElevatorConstants {
        public static final int LEFT_MOTOR_ID = 0;
        public static final int RIGHT_MOTOR_ID = 0;
        public static final double MOTOR_SPEED = 0.25;
    }

    /** Constants for limelight-related data */
    public static final class LimelightConstants {
        /** Spams "Bad LL 2D/3D Pose Data" when no data is coming from the NetworkTableInstance for LL */
        public static final boolean SPAM_BAD_DATA = false;
        /** Name of the front-facing limelight (shooter / AprilTags) */
        public static final String SHOOTER_LLIGHT = "limelight-three";
        /** Name of the back-facing limelight (Intake / Note Detection) */
        public static final String INTAKE_LLIGHT = "limelight-two";

        /** The pipeline that sees all tags */
        public static final int DEFAULT_PIPELINE = 0;
        /** The pipeline that filters for SPEAKER tags */
        public static final int SPEAKER_PIPELINE = 1;
    }

    /** Constants used with the LEDSubsystem */
    public static final class LEDConstants {
        /** Port that the LED strip is plugged into */
        public static final int LED_PORT = 7;
        /** Number of LEDs to iterate through */
        public static final int LED_COUNT = 150;
    }

    /** Constants for the kinematics and driving of the swerve system */
    public static final class SwerveKinematics {
        /** Distance between wheel positions */
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(RobotConstants.WHEEL_BASE / 2, -RobotConstants.TRACK_WIDTH / 2),
            new Translation2d(RobotConstants.WHEEL_BASE / 2, RobotConstants.TRACK_WIDTH / 2),
            new Translation2d(-RobotConstants.WHEEL_BASE / 2, -RobotConstants.TRACK_WIDTH / 2),
            new Translation2d(-RobotConstants.WHEEL_BASE / 2, RobotConstants.TRACK_WIDTH / 2));

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
        public static final double DRIVE_SPEED_COEFFICENT = 3;
        /** The rate of change limit in units per second for driving limiters (x and y) */
        public static final double DRIVE_SLEW_RATE_LIMIT = 1.75;

        /** Multiplies joystick and turning input by the specified coefficient */
        public static final double FINE_CONTROL_COEFFICENT = 0.25;

        /** x and y speed set by the directional pad in meters per second */
        public static final double D_PAD_SPEED = 1;

        /** PID constants used for controlling the turning position of the swerve modules */
        public static final class TURNING_PID_CONTROLLER {
            public static final double KP = 0.325;
            public static final double KI = 0;
            public static final double KD = 0;
        }
    }

    /** Constants for the swerve modules */
    public static final class SwerveModuleConfigs {
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
    }
}
