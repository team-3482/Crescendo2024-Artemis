// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OrbitConstants;
import frc.robot.Constants.SwerveKinematics;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.limelight.LimelightSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utilities.Telemetry;

/** An command to turn the bot until the speaker AprilTag is centered in front of the shooter. */
public class CenterSpeakerCommand extends Command {
    // Instances of Rate Limiters to ensure that the robot moves smoothly
    private final SlewRateLimiter turningLimiter;
    /** Turning {@link PIDController}, uses DEGREES for calculations */
    private PIDController pid;
    /** Used for {@link CenterSpeakerCommand#isFinished()} */
    private double errorRadians;
    /** Used to avoid repeated calls to DS API */
    private Optional<Alliance> alliance;

    public CenterSpeakerCommand() {
        setName("CenterSpeakerCommand");
        
        this.turningLimiter = new SlewRateLimiter(SwerveKinematics.TURNING_SLEW_RATE_LIMIT);
        this.pid = new PIDController(
            OrbitConstants.TURNING_SPEED_PID_CONTROLLER.KP,
            OrbitConstants.TURNING_SPEED_PID_CONTROLLER.KI,
            OrbitConstants.TURNING_SPEED_PID_CONTROLLER.KD);
        this.pid.setTolerance(Units.degreesToRadians(OrbitConstants.TURNING_SPEED_PID_CONTROLLER.TOLERANCE));
        this.pid.enableContinuousInput(0, 360);
        
        // Adds the swerve subsyetm to requirements to ensure that it is the only class
        // modifying its data at a single time
        // Do not require limelight subsystem because it is getter functions only
        addRequirements(SwerveSubsystem.getInstance()); 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.errorRadians = OrbitConstants.TURNING_SPEED_PID_CONTROLLER.TOLERANCE + 1;
        this.pid.reset();
        this.alliance = DriverStation.getAlliance();
        
        LEDSubsystem.getInstance().setLightState(LightState.AUTO_RUNNING);
    }

    @Override
    public void execute() {
        // Skip loops when the LL is not getting proper data, otherwise errorDegrees is 0
        if (LimelightSubsystem.getInstance().getTargetID() != (alliance.get() == Alliance.Red ? 7 : 4)) return;

        this.errorRadians = Units.degreesToRadians(
            LimelightSubsystem.getInstance().getHorizontalOffset(LimelightConstants.SHOOTER_LLIGHT));
        
        double turningSpeed = pid.calculate(this.errorRadians, 0);
        turningSpeed = turningLimiter.calculate(turningSpeed) * SwerveKinematics.TURNING_SPEED_COEFFIECENT;
        
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 
            MathUtil.clamp(turningSpeed, -1.0, 1.0));
        
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
        this.pid.close();
        
        Telemetry.logMessage(this.getName() + (interrupted ? " interrupted" : " ended"), interrupted);
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    /**
    * Ends the command when the error is smaller than tolerance at {@link OrbitConstants}
    */
    @Override
    public boolean isFinished() {
        return Math.abs(this.errorRadians) <= Units.degreesToRadians(OrbitConstants.TURNING_SPEED_PID_CONTROLLER.TOLERANCE);
        // return this.pid.atSetpoint();
    }
}
