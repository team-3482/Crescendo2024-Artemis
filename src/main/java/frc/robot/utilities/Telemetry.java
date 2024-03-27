// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.text.DecimalFormat;
import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants.LimelightConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.ShuffleboardTabConstants;
import frc.robot.constants.Constants.AutonConstants.StartingPositions;
import frc.robot.constants.Constants.TelemetryConstants.LoggingMode;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.limelight.LimelightSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.swerve.SwerveSubsystem;

/** A class used to manage all telemetry components */
public class Telemetry {
    // Singleton Design Pattern
    private static Telemetry instance;

    public static Telemetry getInstance() {
        if (instance == null) {
            instance = new Telemetry();
        }
        return instance;
    }

    /** What to round decimal values to on Shuffleboard */
    public static final DecimalFormat D_FORMAT = new DecimalFormat("#.##");
    /**
     * Toggle for displaying timestamps using {@link Telemetry#logMessage(String, boolean)}
     * @apiNote If {@link Telemetry#initialize()} has not run, this will be {@code null} </p>
     */
    private static GenericEntry LOG_TIMESTAMPS = null;
    // private static boolean LOG_TIMESTAMPS = false;

    /**
     * Prints the string to the console with a tag and timestamp.
     * 
     * @param message to be printed to the console
     * @param logMode indicates which tag should be used in the console
     * @apiNote if {@link Telemetry#LOG_TIMESTAMPS} is null, it will never log timestamps
     */
    public static void logMessage(String message, LoggingMode logMode) {
        String messageTag = logMode.getTag() + " " +
            (LOG_TIMESTAMPS == null ? false : LOG_TIMESTAMPS.getBoolean(false)
                ? "[" + D_FORMAT.format(Timer.getFPGATimestamp()) + " sec] ": "");
        System.out.println(messageTag + message);
    }
    /**
     * Logs the end of a command with an error tag if the command was interrupted or an info tag if the command ended normally 
     * @param name of the command
     * @param interrupted whether the command was interrupted
     * @param trailing printed after the name and interruption
     */
    public static void logCommandEnd(String name, boolean interrupted, String... trailing) {
        String _trailing = "";
        if (trailing.length > 0) {
            for (String information : trailing) {
                _trailing += "(" + information + ") ";
            }
        }

        logMessage(name + " ended " + _trailing, interrupted ? LoggingMode.ERROR : LoggingMode.INFO);
    }

    /**
     * Prints the string to the console with a tag and timestamp and no error (overloaded)
     * 
     * @param message to be printed to the console
     */
    public static void logMessage(String message) {
        logMessage(message, LoggingMode.INFO);
    }

    /** Runs {@link Telemetry#initialize()} once */
    private Telemetry() {
        initialize();
    }

    /** Initializes telemetry classes */
    public void initialize() {
        AdvantageScopeTelemetry.initialize();
        ShuffleboardTelemetry.initialize();
        
        // Telemetry layout
        ShuffleboardLayout telemetryLayout = Shuffleboard.getTab(ShuffleboardTabConstants.PITTING)
            .getLayout("Swerve Subsystem", BuiltInLayouts.kList)
            .withProperties(Map.of("Label position", "TOP"))
            .withPosition(9, 0)
            .withSize(3, 6);
        LOG_TIMESTAMPS = telemetryLayout.add("Log Timestamps", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(0, 0)
            .withSize(3, 1)
            .getEntry();
    }

    /** Publishes telemetry classes */
    public void publish() {
        AdvantageScopeTelemetry.publish();
        ShuffleboardTelemetry.publish();
    }

    /** A class used to manage all AdvantageScope telemetry components */
    private class AdvantageScopeTelemetry {
        private AdvantageScopeTelemetry() {}

        /** Publishes all static AdvantageScope telemetry */
        public static void initialize() {}
            
        /** Publishes all AdvantageScope telemetry */
        public static void publish() {
            swerveModuleStatesPublisher.set(SwerveSubsystem.getInstance().getModuleStates());
            odometryPosePublisher.set(SwerveSubsystem.getInstance().getPose());
            swerveDesiredStatesPublisher.set(SwerveSubsystem.getInstance().getDesiredStates());
        }

        //Publishers
        private static StructArrayPublisher<SwerveModuleState> swerveModuleStatesPublisher = NetworkTableInstance.getDefault()
            .getTable("AdvantageScope")
            .getStructArrayTopic("SwerveModuleStates", SwerveModuleState.struct)
            .publish();

        private static StructArrayPublisher<SwerveModuleState> swerveDesiredStatesPublisher = NetworkTableInstance.getDefault()
            .getTable("AdvantageScope")
            .getStructArrayTopic("desiredStates", SwerveModuleState.struct)
            .publish();

        private static StructPublisher<Pose2d> odometryPosePublisher = NetworkTableInstance.getDefault()
            .getTable("AdvantageScope")
            .getStructTopic("RobotPose", Pose2d.struct)
            .publish();
    }

    /**
     * Getter for retrieving the staring position
     */
    public StartingPositions getSelectedStartingPosition() {
        return ShuffleboardTelemetry.PITTING_STARTING_POSITION.getSelected();
    }

    /** A class used to manage all Shuffleboard telemetry components */
    private class ShuffleboardTelemetry {
        private ShuffleboardTelemetry() {}

        /** Publishes all static Shuffleboard telemetry */
        public static void initialize() {
            // IntakeSubsystem
            // ShuffleboardLayout intakeSubsystemLayout = Shuffleboard.getTab(ShuffleboardTabConstants.PITTING)
            //     .getLayout("Intake Subsystem", BuiltInLayouts.kList)
            //     .withProperties(Map.of("Label position", "TOP"))
            //     .withPosition(0, 0)
            //     .withSize(3, 6);
            // // Reset the pivot's position
            // intakeSubsystemLayout.add("Reset Position Lower Limit",
            //     Commands.runOnce(() -> {
            //         IntakeSubsystem.getInstance().resetPivotPosition(0);
            //     }).ignoringDisable(true).withName(0 + " deg"))
            //     .withPosition(0, 1)
            //     .withWidget(BuiltInWidgets.kCommand);
            // intakeSubsystemLayout.add("Reset Position Higher Limit",
            //     Commands.runOnce(() -> {
            //         IntakeSubsystem.getInstance().resetPivotPosition(IntakeConstants.IntakeState.IDLE.getAngle());
            //     }).ignoringDisable(true).withName(IntakeConstants.IntakeState.IDLE.getAngle() + " deg"))
            //     .withPosition(0, 2)
            //     .withWidget(BuiltInWidgets.kCommand);
            
            // Shooter Subsystem
            ShuffleboardLayout shooterSubsystemLayout = Shuffleboard.getTab(ShuffleboardTabConstants.PITTING)
                .getLayout("Shooter Subsystem", BuiltInLayouts.kList)
                .withProperties(Map.of("Label position", "TOP"))
                .withPosition(3, 0)
                .withSize(3, 6);
            // Force save positions of the pivot
            shooterSubsystemLayout.add("Force Save Position",
                Commands.runOnce(() -> {
                    double[] pos = ShooterSubsystem.getInstance().getPivotPositions();
                    JSONManager.getInstance().saveShooterPivotPositions(pos[0], pos[1]);
                }).ignoringDisable(true).withName("Save Data"))
                .withPosition(0, 0)
                .withWidget(BuiltInWidgets.kCommand);
            // Reset the pivot's position
            shooterSubsystemLayout.add("Reset Position Lower Limit",
                Commands.runOnce(() -> {
                    ShooterSubsystem.getInstance().resetPivotPosition(ShooterConstants.PIVOT_ANGLE_LIMITS[0]);
                }).ignoringDisable(true).withName(ShooterConstants.PIVOT_ANGLE_LIMITS[0] + " deg"))
                .withPosition(0, 1)
                .withWidget(BuiltInWidgets.kCommand);
            shooterSubsystemLayout.add("Reset Position Higher Limit",
                Commands.runOnce(() -> {
                    ShooterSubsystem.getInstance().resetPivotPosition(ShooterConstants.PIVOT_ANGLE_LIMITS[1]);
                }).ignoringDisable(true).withName(ShooterConstants.PIVOT_ANGLE_LIMITS[1] + " deg"))
                .withPosition(0, 2)
                .withWidget(BuiltInWidgets.kCommand);
            shooterSubsystemLayout.add("Reset Position Vertical",
                Commands.runOnce(() -> {
                    ShooterSubsystem.getInstance().resetPivotPosition(90);
                }).ignoringDisable(true).withName(90 + " deg"))
                .withWidget(BuiltInWidgets.kCommand);
            
            // SwerveSubsystem layout for change starting position
            ShuffleboardLayout swerveSubsystemLayout = Shuffleboard.getTab(ShuffleboardTabConstants.PITTING)
                .getLayout("Swerve Subsystem", BuiltInLayouts.kList)
                .withProperties(Map.of("Label position", "TOP"))
                .withPosition(6, 0)
                .withSize(3, 6);
            // Chooser for resetting position
            PITTING_STARTING_POSITION = new SendableChooser<StartingPositions>();
            for (StartingPositions position : StartingPositions.getStartingPositions()) {
                PITTING_STARTING_POSITION.addOption(position.getName(), position);
                if (position.equals(StartingPositions.AUTO)) {
                    PITTING_STARTING_POSITION.setDefaultOption(position.getName(), position);
                }
            }
            PITTING_STARTING_POSITION.onChange((StartingPositions position) -> {
                Pose2d startingPose = SwerveUtilities.getStartingPose(PITTING_STARTING_POSITION.getSelected());
                SwerveSubsystem.getInstance().setHeading(startingPose.getRotation().getDegrees());
                SwerveSubsystem.getInstance().setPose(startingPose);
            });
            swerveSubsystemLayout.add("Starting Position", PITTING_STARTING_POSITION)
                .withWidget(BuiltInWidgets.kComboBoxChooser);
            // Re-set chosen position
            swerveSubsystemLayout.add("Set Starting Position",
                Commands.runOnce(() -> {
                    Pose2d startingPose = SwerveUtilities.getStartingPose(PITTING_STARTING_POSITION.getSelected());
                    SwerveSubsystem.getInstance().setHeading(startingPose.getRotation().getDegrees());
                    SwerveSubsystem.getInstance().setPose(startingPose);
                }).ignoringDisable(true).withName("Set Again"))
                .withWidget(BuiltInWidgets.kCommand);
            
            // LimelightSubsystem
            HttpCamera limelightFrontFeed = new HttpCamera(
                LimelightConstants.SHOOTER_LLIGHT,
                "http://" + LimelightConstants.SHOOTER_LLIGHT + ".local:5800/stream.mjpg",
                HttpCameraKind.kMJPGStreamer);
            CameraServer.addCamera(limelightFrontFeed);
            Shuffleboard.getTab(ShuffleboardTabConstants.DEFAULT)
                .add("Front Limelight", limelightFrontFeed)
                .withWidget(BuiltInWidgets.kCameraStream)
                .withPosition(3, 1)
                .withSize(6, 3)
                .withProperties(Map.of("Show Crosshair", false, "Show Controls", false));
            HttpCamera limelightBackFeed = new HttpCamera(
                LimelightConstants.INTAKE_LLIGHT,
                "http://" + LimelightConstants.INTAKE_LLIGHT + ".local:5800/stream.mjpg",
                HttpCameraKind.kMJPGStreamer);
            CameraServer.addCamera(limelightBackFeed);
            Shuffleboard.getTab(ShuffleboardTabConstants.DEFAULT)
                .add("Back Limelight", limelightBackFeed)
                .withWidget(BuiltInWidgets.kCameraStream)
                .withPosition(9, 1)
                .withSize(6, 3)
                .withProperties(Map.of("Show Crosshair", false, "Show Controls", false));
        }

        /** Keeps track of which item to publish next */
        private static int index = 0;

        /** Publishes all Shuffleboard telemetry */
        public static void publish() {
            switch (index) {
                case 0: {
                    // IntakeSubsystem
                    DEFAULT_INTAKE_PIVOT_POSITION.setString(Telemetry.D_FORMAT.format(IntakeSubsystem.getInstance().getPivotPosition()));
                    break;
                }

                case 1: {
                    // ShooterSubsystem
                    double[] pos = ShooterSubsystem.getInstance().getPivotPositions();
                    DEFAULT_SHOOTER_PIVOT_POSITIONS.setString("Left    " + Telemetry.D_FORMAT.format(pos[0]) + "    ||    " + Telemetry.D_FORMAT.format(pos[1]) + "    Right");
                    break;
                }

                case 2: {
                    // Swerve Subsystem
                    DEFAULT_GYRO_HEADING.setDouble(SwerveSubsystem.getInstance().getHeading());
                    break;
                }

                case 3: {
                    // LimelightSubsystem
                    DEFAULT_LIMELIGHT_ODOMETRY.setBoolean(SwerveSubsystem.getInstance().usingLimelightOdometry());
                    DEFAULT_LIMELIGHT_TARGET_ID.setInteger(LimelightSubsystem.getInstance().getTargetID());
                    break;
                }
            }
            
            index = index == 3 ? 0 : index + 1;
        }

        // LEDSubsystem
        // Staying in LEDSubsystem.java for blinking UNLESS it impacts loop time
        // TODO check loop times for LEDSubsystem.periodic()

        // IntakeSubsystem
        private static GenericEntry DEFAULT_INTAKE_PIVOT_POSITION = Shuffleboard.getTab(ShuffleboardTabConstants.DEFAULT)
            .add("Intake Pivot", "")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(9, 0)
            .withSize(2, 1)
            .getEntry();
        
        // ShooterSubsystem
        private static GenericEntry DEFAULT_SHOOTER_PIVOT_POSITIONS = Shuffleboard.getTab(ShuffleboardTabConstants.DEFAULT)
            .add("Shooter Pivot", "")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(6, 0)
            .withSize(3, 1)
            .getEntry();
        
        // SwerveSubsystem
        private static GenericEntry DEFAULT_GYRO_HEADING = Shuffleboard.getTab(ShuffleboardTabConstants.DEFAULT)
            .add("Robot Heading", 0)
            .withWidget(BuiltInWidgets.kGyro)
            .withProperties(Map.of("Starting angle", 180, "Counter Clockwise", true))
            .withPosition(0, 0)
            .withSize(3, 3)
            .getEntry();
        public static SendableChooser<StartingPositions> PITTING_STARTING_POSITION;
        
        // LimelightSubsystem
        private static GenericEntry DEFAULT_LIMELIGHT_TARGET_ID = Shuffleboard.getTab(ShuffleboardTabConstants.DEFAULT)
            .add("T ID", -1)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(3, 0)
            .withSize(1, 1)
            .getEntry();
        
        private static GenericEntry DEFAULT_LIMELIGHT_ODOMETRY = Shuffleboard.getTab(ShuffleboardTabConstants.DEFAULT)
            .add("Vision Odometry", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(4, 0)
            .withSize(2, 1)
            .getEntry();
    }
}