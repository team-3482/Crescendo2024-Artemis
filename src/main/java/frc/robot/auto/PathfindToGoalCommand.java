// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;

/** A pathfinding command that uses limelight and swerve subsystems. */
public class PathfindToGoalCommand extends Command {
    private final PathConstraints CONSTRAINTS = new PathConstraints(
        AutonConstants.MAX_LINEAR_VELOCITY,
        AutonConstants.MAX_LINEAR_ACCELERATION,
        AutonConstants.MAX_ANGULAR_VELOCITY,
        AutonConstants.MAX_ANGULAR_ACCELERATION);
    
    private Character goal;
    private Command path;

    /**
    * Creates a new BezierToGoalCommand.
    *
    * @param swerveSubsystem The swerve subsystem used by this command.
    * @param goal The location to line up at
    */
    public PathfindToGoalCommand(Character goal) {
        this.goal = goal;

        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent() || this.path != null) {
            LEDSubsystem.getInstance().setLightState(LightState.WARNING);
            return;
        }
        LEDSubsystem.getInstance().setLightState(LightState.SOLID_ORANGE);
        
        Pose2d targetPose = AutonConstants.IDEAL_TAG_POSITIONS.get(alliance.get()).get(this.goal);
        
        this.path = AutoBuilder.pathfindToPose(targetPose, CONSTRAINTS, 0, 0.1);
        this.path.schedule();
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
        this.path = null;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (this.path != null) {
            return this.path.isFinished();
        }
        return true;
    }
}
