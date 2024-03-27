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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Positions;
import frc.robot.constants.Constants.AutonConstraints;
import frc.robot.constants.Positions.PathfindingPosition;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.swerve.SwerveSubsystem;

/** A class that generates pathfinding commands. */
public final class PathingCommands {
    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        AutonConstraints.MAX_LINEAR_VELOCITY,
        AutonConstraints.MAX_LINEAR_ACCELERATION,
        AutonConstraints.MAX_ANGULAR_VELOCITY,
        AutonConstraints.MAX_ANGULAR_ACCELERATION);
    
    /**
     * This class should only be used for static getters. Do not initialize it.
     * @throws Exception This class should not be initialized
     */
    private PathingCommands() throws Exception {
        throw new Exception("This class should not be initialized");
    }

    /**
     * Calculates a position using {@link Autobuilder#pathfindToPose()}
     * @param targetPosition
     * @return the Pathfinding command or {@link Commands#none()} if no alliance is found
     */
    public static Command getPathfindCommand(PathfindingPosition targetPosition) {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            LEDSubsystem.getInstance().setLightState(LightState.WARNING);
            return Commands.none();
        }
        
        Pose2d targetPose = Positions.PATHFIND_POSITIONS.get(alliance.get()).get(targetPosition);
        
        Command path = AutoBuilder.pathfindToPose(targetPose, PathingCommands.CONSTRAINTS, 0, 0.1);
        path.setName("PathfindCommand");

        LEDSubsystem.getInstance().setLightState(LightState.AUTO_RUNNING);
        return path;
    }

    /**
     * Calculates a position using {@link PathPlannerPath#bezierFromPoses(Pose2d...)}
     * @param targetPosition
     * @return the Bezier command or {@link Commands#none()} if no alliance is found
     */
    public static Command getBezierCommand(PathfindingPosition targetPosition) {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            LEDSubsystem.getInstance().setLightState(LightState.WARNING);
            return Commands.none();
        }
        
        Pose2d botPose = SwerveSubsystem.getInstance().getPose();
        // The rotation component for endPos is used for the GoalEndState rotation
        Pose2d endPos = Positions.PATHFIND_POSITIONS.get(alliance.get()).get(targetPosition);
        // The travelRotation represents the direction of travel
        Rotation2d travelRotation = endPos.getRotation();
        
        Pose2d startPos = new Pose2d(botPose.getTranslation(), travelRotation);
        
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);

        PathPlannerPath bezierPath = new PathPlannerPath(
            bezierPoints,
            PathingCommands.CONSTRAINTS,
            new GoalEndState(0, endPos.getRotation())
        );
        // Prevent this path from being flipped on the red alliance, since the given positions are already correct
        bezierPath.preventFlipping = true;
    
        Command path = AutoBuilder.followPath(bezierPath);
        path.setName("BezierCommand");
        
        return path;
    }
}
