// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.SwerveSubsystem;

/** A pathfinding command that uses limelight and swerve subsystems. */
public class BezierToGoalCommand extends Command {
    private final PathConstraints CONSTRAINTS = new PathConstraints(
        AutonConstants.MAX_LINEAR_VELOCITY,
        AutonConstants.MAX_LINEAR_ACCELERATION,
        AutonConstants.MAX_ANGULAR_VELOCITY,
        AutonConstants.MAX_ANGULAR_ACCELERATION);
    
    private final SwerveSubsystem swerveSubsystem;
    private Character goal;

    /**
    * Creates a new PathfindAprilTagCommand.
    *
    * @param swerveSubsystem The swerve subsystem used by this command.
    * @param goal The location to line up at
    */
    public BezierToGoalCommand(SwerveSubsystem swerveSubsystem, Character goal) {
        this.swerveSubsystem = swerveSubsystem;
        this.goal = goal;

        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) return;

        // The rotation component in these poses represents the direction of travel
        Pose2d startPos = new Pose2d(swerveSubsystem.getPose().getTranslation(), new Rotation2d());
        Pose2d endPos = AutonConstants.IDEAL_TAG_POSITIONS.get(alliance.get()).get(this.goal);
        
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);

        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            this.CONSTRAINTS,
            new GoalEndState(0, new Rotation2d())
        );
    
        // Prevent this path from being flipped on the red alliance, since the given positions are already correct
        path.preventFlipping = true;
    
        AutoBuilder.followPath(path).schedule();

        this.cancel();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
