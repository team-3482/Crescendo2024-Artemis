// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.SwerveKinematics;
import frc.robot.subsystems.LimelightSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class PathfindAprilTagCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final LimelightSubsystem limelightSubsystem;
    private Command path;
    private boolean noPath;

    /**
    * Creates a new ExampleCommand.
    *
    * @param subsystem The subsystem used by this command.
    */
    public PathfindAprilTagCommand(LimelightSubsystem subsystem) {
        this.limelightSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double xPos = limelightSubsystem.getXPos();
        double yPos = limelightSubsystem.getYPos();

        if (xPos == 0 && yPos == 0) {
            this.noPath = true;
            return;
        }
        this.noPath = false;

        Pose2d aprilTagPosition = new Pose2d(
            xPos,
            yPos,
            Rotation2d.fromDegrees(limelightSubsystem.getAngle()));

        PathConstraints constraints = new PathConstraints(
            AutonConstants.MAX_DRIVE_SPEED_METERS_PER_SECOND_AUTON,
            AutonConstants.MAX_DRIVE_ACCELERATION_METERS_PER_SECOND_SQUARED_AUTON,
            SwerveKinematics.MAX_DRIVE_ANGULAR_SPEED_RADIANS_PER_SECOND,
            SwerveKinematics.MAX_TURN_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        this.path = AutoBuilder.pathfindToPose(
            aprilTagPosition,
            constraints,
            0.0, // Goal end velocity in meters/sec
            0.0); // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        
        path.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (path != null) {
            path.end(interrupted);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (path != null) {
            return path.isFinished();
        }
        return this.noPath;
    }
}
