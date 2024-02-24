// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OrbitConstants;
import frc.robot.Constants.SwerveKinematics;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;

public class CenterSpeakerCommand extends Command {
    // Instances of Rate Limiters to ensure that the robot moves smoothly
    private final SlewRateLimiter turningLimiter;
    private PIDController rotationPidController;
    private boolean finished;

    public CenterSpeakerCommand() {
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
        LEDSubsystem.getInstance().setLightState(LightState.SOLID_GREEN);
        Translation2d point = OrbitConstants.ORBIT_POINT.get(alliance.get());
        
        // Orbit calculations
        Translation2d difference = SwerveSubsystem.getInstance().getPose().getTranslation().minus(point);
        
        // y is negative when the angle has to be positive and vice versa so it has to be reversed
        double angleGoalRad = Math.atan2(difference.getX(), - difference.getY()) + Math.PI / 2;
        System.out.println(angleGoalRad);
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
