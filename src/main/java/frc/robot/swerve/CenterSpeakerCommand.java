// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OrbitConstants;
import frc.robot.Constants.SwerveKinematics;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.limelight.LimelightSubsystem;

/** An command to turn the bot until the speaker AprilTag is centered in front of the shooter. */
public class CenterSpeakerCommand extends Command {
    // Instances of Rate Limiters to ensure that the robot moves smoothly
    private final SlewRateLimiter turningLimiter;
    private PIDController rotationPidController;
    private boolean finished;

    public CenterSpeakerCommand() {
        setName("CenterSpeakerCommand");
        this.turningLimiter = new SlewRateLimiter(OrbitConstants.ORBIT_TURNING_SLEW_RATE_LIMIT);
        this.rotationPidController = new PIDController(
            OrbitConstants.TURNING_SPEED_PID_CONTROLLER.KP,
            OrbitConstants.TURNING_SPEED_PID_CONTROLLER.KI,
            OrbitConstants.TURNING_SPEED_PID_CONTROLLER.KD);
        this.rotationPidController.setTolerance(OrbitConstants.TURNING_SPEED_PID_CONTROLLER.TOLERANCE);
        this.rotationPidController.enableContinuousInput(0, 2 * Math.PI);
        
        // Adds the swerve subsyetm to requirements to ensure that it is the only class
        // modifying its data at a single time
        // Do not require limelight subsystem because it is getter functions only
        this.addRequirements(SwerveSubsystem.getInstance()); 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.finished = false;
        rotationPidController.reset();
        LEDSubsystem.getInstance().setLightState(LightState.CMD_INIT);
    }

    @Override
    public void execute() {
        // Double so it can be null if the ID cannot be orbited
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            LEDSubsystem.getInstance().setLightState(LightState.WARNING);
            this.finished = true;
            return;
        }
        LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);
        Translation3d _point = OrbitConstants.ORBIT_POINT.get(alliance.get());
        Translation2d point = new Translation2d(_point.getX(), _point.getY());
        
        // Orbit calculations
        double angleGoalRad;
        if (LimelightSubsystem.getInstance().hasTarget(LimelightConstants.SHOOTER_LLIGHT)) {
            double errorDegrees = LimelightSubsystem.getInstance().getHorizontalOffset(LimelightConstants.SHOOTER_LLIGHT);
            angleGoalRad = Units.degreesToRadians(errorDegrees);
            System.out.println("goal limelight " + errorDegrees);
        }
        else { // Position
            Translation2d difference = SwerveSubsystem.getInstance().getPose().getTranslation().minus(point);
            // double angleGoalRad = Math.atan2(difference.getX(), - difference.getY()) + Math.PI / 2;
            angleGoalRad = Math.PI - Math.atan2(difference.getY(), difference.getX());
            System.out.println("goal odometry " + Units.radiansToDegrees(angleGoalRad));
        }

        this.finished = Math.abs(angleGoalRad) <= OrbitConstants.TURNING_SPEED_PID_CONTROLLER.TOLERANCE;
        
        double turningSpeed = rotationPidController
            .calculate(Units.degreesToRadians(SwerveSubsystem.getInstance().getHeading()), angleGoalRad);
        
        turningSpeed = turningLimiter.calculate(turningSpeed) * SwerveKinematics.TURNING_SPEED_COEFFIECENT;
        
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, turningSpeed);
        
        SwerveSubsystem.getInstance().setChassisSpeeds(chassisSpeeds);
    }

    /**
    * Makes the swerve modules stop when the command ends or is interrupted
    * 
    * @param interrupted
    */
    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().stopModules();
        rotationPidController.close();
        if (interrupted) {
            LEDSubsystem.getInstance().setLightState(LightState.WARNING);
        }
        else {
            LEDSubsystem.getInstance().setLightState(LightState.OFF);
        }
        this.finished = false;
    }

    /**
    * Returns false because this command should run until the user releases the button
    * 
    * @return boolean - always false
    */
    @Override
    public boolean isFinished() {
        System.out.println(finished);
        return this.finished;
    }
}
