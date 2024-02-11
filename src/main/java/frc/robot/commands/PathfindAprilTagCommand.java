// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** A pathfinding command that uses limelight and swerve subsystems. */
public class PathfindAprilTagCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final LimelightSubsystem limelightSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private Command path;

    private boolean noPath;
    /**
    * Creates a new PathfindAprilTagCommand.
    *
    * @param limelightSubsystem The limelight subsystem used by this command.
    * @param swerveSubsystem The swerve subsystem used by this command.
    */
    public PathfindAprilTagCommand(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        // Do not require limelight subsystem because it is getter functions only
        addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // int tagID = limelightSubsystem.getID();
        // Translation2d idealPositionTrans = AutonConstants.IDEAL_TAG_POSITIONS.get(tagID);
        
        // if (tagID <= 0 || idealPositionTrans == null) {
        if (!DriverStation.getAlliance().isPresent()) {
            this.noPath = true;
            return;
        }
        this.noPath = false;    
        
        // Pose2d idealPosition = new Pose2d(idealPositionTrans,
        //     new Rotation2d(Math.atan2(idealPositionTrans.getY(), idealPositionTrans.getX())));
        Pose2d idealPosition = DriverStation.getAlliance().get() == Alliance.Red ?
            AutonConstants.IDEAL_TAG_POSITIONS.get(4) :
            AutonConstants.IDEAL_TAG_POSITIONS.get(7);

        PathConstraints constraints = new PathConstraints(
            AutonConstants.MAX_LINEAR_VELOCITY,
            AutonConstants.MAX_LINEAR_ACCELERATION,
            AutonConstants.MAX_ANGULAR_VELOCITY,
            AutonConstants.MAX_ANGULAR_ACCELERATION);

        this.path = AutoBuilder.pathfindToPose(
            idealPosition,
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
            path.cancel();
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
