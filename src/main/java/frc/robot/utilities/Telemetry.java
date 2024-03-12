// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
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

    // TODO implement
    /** A class used to manage all Shuffleboard telemetry components */
    private class ShuffleboardTelemetry {
        private ShuffleboardTelemetry() {}

        /** Publishes all one-time Shuffleboard telemetry */
        public static void initialize() {
            // TODO buttons and such
        }

        /** Publishes all Shuffleboard telemetry */
        public static void publish() {
            // TODO data updates
        }

        // TODO variables
    }
}