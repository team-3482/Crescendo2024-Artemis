// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants.TelemetryConstants.LoggingTags;
import frc.robot.constants.PhysicalConstants.IntakeConstants;
import frc.robot.constants.PhysicalConstants.ShooterConstants;
import frc.robot.constants.PhysicalConstants.SwerveKinematics;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utilities.Telemetry;

/** Constants used throughout the code that are not categorized in other constants files. */
public final class Constants {
    /** Stores all shooter configuration related data */
    public static enum ShooterStates {
        FRONT_EJECT(false, false, false, ShooterConstants.Pivot.ANGLE_LIMITS[0], 25.0, 750.0),
        INTAKE(false, true, false, ShooterConstants.Pivot.ANGLE_LIMITS[0], null, null),
        AMP(false, true, false, ShooterConstants.Pivot.ANGLE_LIMITS[1], 10.0, 435.0),
        SPEAKER(false, true, false, ShooterConstants.Pivot.ANGLE_LIMITS[1], 75.0, 1500.0),
        SPEAKER_CALCULATE(true, true, true, null, 100.0, 1500.0, 1900.0),
        MANUAL(false, false, false, null, 100.0, 1800.0)
        ;

        boolean calculateAngle;
        boolean autoEndShooting;
        boolean spin;
        Double positionAngle;
        Double[] rpms;
        Double allowedError;

        /**
         * Creates a new ShooterState
         * @param calculateAngle
         * @param autoEndShooting
         * @param spin
         * @param angle
         * @param allowedError
         * @param RPMS - Put lower RPM first and higher second
         */
        private ShooterStates(boolean calculateAngle, boolean autoEndShooting, boolean spin,
            Double angle, Double allowedError, Double... RPMS) {
            this.calculateAngle = calculateAngle;
            this.autoEndShooting = autoEndShooting;
            this.spin = spin;
            this.positionAngle = angle;
            this.allowedError = allowedError;
            this.rpms = RPMS;
        }

        /**
         * Whether or not to automatically calculate shooting angles
         * @return calculateAngle
         */
        public boolean getCalculateAngle() {
            return this.calculateAngle;
        }

        /**
         * Whether or not to stop the sterilizer when no note is detected
         * @return autoEndShooting
         */
        public boolean getAutoEndShooting() {
            return this.autoEndShooting;
        }

        /**
         * The angle the shooter should be at for this state
         * @return the angle
         */
        public double getAngle() {
            return this.positionAngle;
        }

        /**
         * The RPMs for this state
         * @param invert inverts highRPM in [0] and [1]
         * @return the RPMs
         */
        public double[] getRPMs(boolean invert) {
            double _rpms = this.rpms[0];

            if (this.calculateAngle) {
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    Translation3d point = Positions.SPEAKER_TARGETS.get(DriverStation.getAlliance().get());
                    Pose2d botpose = SwerveSubsystem.getInstance().getPose();
                    
                    // Calculates horizontal distance to speaker
                    double dist = Math.sqrt(
                        Math.pow(point.getX() - botpose.getX(), 2) +
                        Math.pow(point.getY() - botpose.getY(), 2)
                    );

                    if (dist <= 1.5) {
                        _rpms = this.rpms[0];
                    }
                    else {
                        _rpms = this.rpms[1];
                    }
                }
                else {
                    Telemetry.logMessage("DriverStation alliance is not present", LoggingTags.ERROR);
                }
            }

            return new double[]{
                _rpms * (this.spin && !invert ? (double) 1 : 1),
                _rpms * (this.spin && invert ? (double) 1 : 1)
            };
        }

        /**
         * The tolerance for this state's angle
         * @return the allowed error
         */
        public double getAllowedError() {
            return this.allowedError;
        }
    }
    
    /** Stores all intake configuration related data */
    public static enum IntakeStates {
        INTAKING(0, 5, IntakeConstants.INTAKE_SPEED),
        /** The hardware stop angle for the intake when it is idle in degrees*/
        IDLE(148, 5, 0)
        ;
        
        /* Angle of the intake in degrees */
        double intakeAngle;
        double tolerance;
        double speed;

        private IntakeStates(double intakeAngle, double tolerance, double speed) {
            this.intakeAngle = intakeAngle;
            this.tolerance = tolerance;
            this.speed = speed;
        }
        /**
         * The angle of this state
         * @return the angle
         */
        public double getAngle() {
            return this.intakeAngle;
        }
        /**
         * The tolerance for movement to this angle
         * @return the tolerance
         */
        public double getTolerance() {
            return this.tolerance;
        }
        /**
         * The speed at which to spin the intake
         * @return the speed
         */
        public double getSpeed() {
            return this.speed;
        }
    }

    /** Tab names in Shuffleboard */
    public static final class ShuffleboardTabNames {
        public static final String DEFAULT = "Competition";
        public static final String UTILITIES = "Utilities";
    }

    public static final class TelemetryConstants {
        public static enum LoggingTags {
            INFO("INFO"),
            ERROR("ERROR");

            String tag;
            private LoggingTags(String tag) {
                this.tag = tag;
            }
            public String getTag()
            {
                return "[" + this.tag + "]";
            }
        }    
    }

    /** Values used for running autonomous code */
    public static final class AutonConstraints {
        // These are used for on-the-fly paths
        public static final double MAX_LINEAR_VELOCITY = SwerveKinematics.DRIVE_SPEED_COEFFICENT;
        public static final double MAX_LINEAR_ACCELERATION = SwerveKinematics.DRIVE_SLEW_RATE_LIMIT;
        public static final double MAX_ANGULAR_VELOCITY = SwerveKinematics.TURNING_SPEED_COEFFIECENT;
        public static final double MAX_ANGULAR_ACCELERATION = SwerveKinematics.TURNING_SLEW_RATE_LIMIT / 3;
    }

    /** Constants for autos that use the shooter limelight */
    public static final class AprilTagConstants {
        /** PID constants for controlling the turning speed during centering */
        public static final class PPID {
            /** Tolerance for the PID controller in degrees */
            public static final double TOLERANCE = 3;
            public static final double KP_HIGH = 1;
            public static final double KP_LOW = 1.5;
            public static final double KP_LOWEST = 3.5;
            public static final double KI = 0;
            public static final double KD = 0;
        }

        public static final double TURNING_SPEED_COEFFIECENT = SwerveKinematics.TURNING_SPEED_COEFFIECENT;
        public static final double TURNING_SLEW_RATE_LIMIT = SwerveKinematics.TURNING_SLEW_RATE_LIMIT;
    }

    /** Constants for autos that use the intake limelight */
    public static final class NoteConstants {
        /** The rate of change limit (units per second) for turning limiter in orbit mode */
        public static final double NOTE_TURNING_SLEW_RATE_LIMIT = SwerveKinematics.TURNING_SLEW_RATE_LIMIT;
        /** The rate limit in units per second for driving in orbit mode (x and y) */
        public static final double NOTE_DRIVE_SLEW_RATE_LIMIT = SwerveKinematics.DRIVE_SLEW_RATE_LIMIT;
        /** The input speed the bot should have when driving to a note (between 0.0 and 1.0) */
        public static final double NOTE_DRIVE_INPUT_SPEED = (double) 1 / 3;

        /** Time limit for the centering command in seconds */
        public static final double CENTERING_TIMEOUT = 1.5;

        /** PID constants for controlling the turning speed during centering */
        public static final class PID {
            /** Tolerance for the PID controller in degrees */
            public static final double TOLERANCE = 5;
            public static final double KP = 0.6;
            public static final double KI = 0;
            public static final double KD = 0;
        };
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
}
