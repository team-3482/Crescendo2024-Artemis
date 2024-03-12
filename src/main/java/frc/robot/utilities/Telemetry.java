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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShuffleboardTabConstants;
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

    /** Runs {@link Telemetry#initialize()} and {@link Telemetry#publish()} once */
    private Telemetry() {
        initialize();
        publish();
    }

    /** Initializes telemetry classes */
    public void initialize() {
        AdvantageScopeTelemetry.initialize();
        ShuffleboardTelemetry.initialize();
    }

    /** Publishes telemetry classes */
    public void publish() {
        AdvantageScopeTelemetry.publish();
        ShuffleboardTelemetry.publish();
    }

    /** A class used to manage all AdvantageScope telemetry components */
    private class AdvantageScopeTelemetry {
        private AdvantageScopeTelemetry() {}

        /** Publishes all one-time AdvantageScope telemetry */
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

    /** A class used to manage all Shuffleboard telemetry components */
    private class ShuffleboardTelemetry {
        private ShuffleboardTelemetry() {}

        /** Publishes all one-time Shuffleboard telemetry */
        public static void initialize() {
            // IntakeSubsystem
            ShuffleboardLayout intakeSubsystemLayout = Shuffleboard.getTab(ShuffleboardTabConstants.PITTING)
                .getLayout("Intake Pivot", BuiltInLayouts.kList)
                .withProperties(Map.of("Label position", "TOP"))
                .withPosition(3, 0)
                .withSize(3, 6);
            // Reset the pivot's position
            intakeSubsystemLayout.add("Reset Position Lower Limit",
                Commands.runOnce(() -> {
                    IntakeSubsystem.getInstance().resetPivotPosition(0);
                }).ignoringDisable(true).withName(0 + " deg"))
                .withPosition(0, 1)
                .withWidget(BuiltInWidgets.kCommand);
            intakeSubsystemLayout.add("Reset Position Higher Limit",
                Commands.runOnce(() -> {
                    IntakeSubsystem.getInstance().resetPivotPosition(IntakeConstants.IntakeState.IDLE.getAngle());
                }).ignoringDisable(true).withName(IntakeConstants.IntakeState.IDLE.getAngle() + " deg"))
                .withPosition(0, 2)
                .withWidget(BuiltInWidgets.kCommand);
            
            // Shooter Subsystem
            ShuffleboardLayout shooterSubsystemLayout = Shuffleboard.getTab(ShuffleboardTabConstants.PITTING)
                .getLayout("Shooter Pivot", BuiltInLayouts.kList)
                .withProperties(Map.of("Label position", "TOP"))
                .withPosition(0, 0)
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
                    ShooterSubsystem.getInstance().resetPivotPosition(ShooterConstants.PIVOT_ANGLE_LIMITS[0]);
                }).ignoringDisable(true).withName(ShooterConstants.PIVOT_ANGLE_LIMITS[1] + " deg"))
                .withPosition(0, 2)
                .withWidget(BuiltInWidgets.kCommand);
            shooterSubsystemLayout.add("Reset Position Vertical",
                Commands.runOnce(() -> {
                    ShooterSubsystem.getInstance().resetPivotPosition(90);
                }).ignoringDisable(true).withName(90 + " deg"))
                .withWidget(BuiltInWidgets.kCommand);
            
            // TODO
            // SwerveSubsystem layout for change starting position
            
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

        /** Publishes all Shuffleboard telemetry */
        public static void publish() {
            // IntakeSubsystem
            DEFAULT_INTAKE_PIVOT_POSITION.setString(D_FORMAT.format(IntakeSubsystem.getInstance().getPivotPosition()));

            // ShooterSubsystem
            double[] pos = ShooterSubsystem.getInstance().getPivotPositions();
            DEFAULT_SHOOTER_PIVOT_POSITIONS.setString("Left    " + D_FORMAT.format(pos[0]) + "    ||    " + D_FORMAT.format(pos[1]) + "    Right");

            // Swerve Subsystem
            SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
            DEFAULT_GYRO_HEADING.setDouble(swerveSubsystem.getHeading());

            // LimelightSubsystem
            DEFAULT_LIMELIGHT_ODOMETRY.setBoolean(swerveSubsystem.usingLimelightOdometry());
            DEFAULT_LIMELIGHT_TARGET_ID.setInteger(LimelightSubsystem.getInstance().getTargetID());
        }

        /** What to round decimal values to on Shuffleboard */
        public static final DecimalFormat D_FORMAT = new DecimalFormat("#.##");

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