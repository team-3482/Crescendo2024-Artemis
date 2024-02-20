// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class Logger extends Command {
    
    //Publishers
    private StructArrayPublisher<SwerveModuleState> swervePublisher = NetworkTableInstance.getDefault()
        .getTable("AdvantageScope")
        .getStructArrayTopic("SwerveModuleStates", SwerveModuleState.struct)
        .publish();

    private StructPublisher<Pose2d> swervePosePublisher = NetworkTableInstance.getDefault()
        .getTable("AdvantageScope")
        .getStructTopic("RobotPose", Pose2d.struct)
        .publish();

    private StructArrayPublisher<SwerveModuleState> swerveDesiredStatesPublisher = NetworkTableInstance.getDefault()
        .getTable("AdvantageScope")
        .getStructArrayTopic("desiredStates", SwerveModuleState.struct)
        .publish();
    
    /*
     * Creates and initializes a new logger object
     */
    public Logger() { }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Publishes the data to the network table
        swervePublisher.set(SwerveSubsystem.getInstance().getModuleStates());
        swervePosePublisher.set(SwerveSubsystem.getInstance().getPose());
        swerveDesiredStatesPublisher.set(SwerveSubsystem.getInstance().getDesiredStates());
    }
}
