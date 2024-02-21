// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

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
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.swerve.SwerveSubsystem;

/** A pathfinding command that uses limelight and swerve subsystems. */
public class BezierToGoalCommand extends Command {
    private final PathConstraints CONSTRAINTS = new PathConstraints(
        AutonConstants.MAX_LINEAR_VELOCITY,
        AutonConstants.MAX_LINEAR_ACCELERATION,
        AutonConstants.MAX_ANGULAR_VELOCITY,
        AutonConstants.MAX_ANGULAR_ACCELERATION);
    
    private Character goal;
    private Command bezierPath;

    /**
    * Creates a new BezierToGoalCommand.
    *
    * @param swerveSubsystem The swerve subsystem used by this command.
    * @param goal The location to line up at
    */
    public BezierToGoalCommand(Character goal) {
        this.goal = goal;

        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        LEDSubsystem.getInstance().setLightState(LightState.SOLID_GREEN);
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent() || this.bezierPath != null) {
            LEDSubsystem.getInstance().setLightState(LightState.WARNING);
            return;
        }
        
        Pose2d botPose = SwerveSubsystem.getInstance().getPose();
        // The rotatin component for endPos is used for the GoalEndState rotation
        Pose2d endPos = AutonConstants.IDEAL_TAG_POSITIONS.get(alliance.get()).get(this.goal);
        // The travelRotation represents the direction of travel
        Rotation2d travelRotation = botPose.getRotation(); // May need to be endPos.getRotation() or new Rotation2d()
        
        Pose2d startPos = new Pose2d(botPose.getTranslation(), travelRotation);
        
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);

        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            this.CONSTRAINTS,
            new GoalEndState(0, endPos.getRotation())
        );
    
        // Prevent this path from being flipped on the red alliance, since the given positions are already correct
        path.preventFlipping = true;
    
        this.bezierPath = AutoBuilder.followPath(path);
        this.bezierPath.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        LEDSubsystem.getInstance().setLightState(LightState.SOLID_BLUE);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            LEDSubsystem.getInstance().setLightState(LightState.WARNING);
        }
        else {
            LEDSubsystem.getInstance().setLightState(LightState.OFF);
        }
        this.bezierPath = null;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (this.bezierPath != null) {
            return this.bezierPath.isFinished();
        }
        return true;
    }
}
