// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import java.util.Optional;

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
import frc.robot.limelight.LimelightHelpers;
import frc.robot.limelight.LimelightSubsystem;

/** An command to turn the bot until the speaker AprilTag is centered in front of the shooter. */
public class CenterSpeakerCommand extends Command {
    // Instances of Rate Limiters to ensure that the robot moves smoothly
    private final SlewRateLimiter turningLimiter;
    /** Turning {@link PIDController}, uses DEGREES for calculations */
    private PIDController pid;
    /** Used for {@link CenterSpeakerCommand#isFinished()} */
    private double errorRadians;

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
        LEDSubsystem.getInstance().setLightState(LightState.CMD_INIT);
        this.errorRadians = OrbitConstants.TURNING_SPEED_PID_CONTROLLER.TOLERANCE + 1;
        this.pid.reset();
        // LimelightHelpers.setPipelineIndex(LimelightConstants.SHOOTER_LLIGHT, LimelightConstants.SPEAKER_PIPELINE);
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // Target 4 when blue or when no alliance is found, or 7 otherwise
        LimelightHelpers.getLimelightNTTableEntry(LimelightConstants.SHOOTER_LLIGHT, "priorityid").setInteger(
            alliance.isPresent() && alliance.get() == Alliance.Red ? 7 : 4
        );
        
        LEDSubsystem.getInstance().setLightState(LightState.AUTO_RUNNING);
    }

    @Override
    public void execute() {
        // Skip loops when the LL is not getting proper data, otherwise errorDegrees is 0
        if (!LimelightSubsystem.getInstance().hasTarget(LimelightConstants.SHOOTER_LLIGHT)) return;

        this.errorRadians = Units.degreesToRadians(
            LimelightSubsystem.getInstance().getHorizontalOffset(LimelightConstants.SHOOTER_LLIGHT));
        
        double turningSpeed = pid.calculate(this.errorRadians, 0);
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
        // LimelightHelpers.setPipelineIndex(LimelightConstants.SHOOTER_LLIGHT, LimelightConstants.DEFAULT_PIPELINE);
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
        this.pid.close();
    }

    /**
    * End logic in {@link CenterSpeakerCommand#execute()} body
    */
    @Override
    public boolean isFinished() {
        return this.errorRadians > Units.degreesToRadians(OrbitConstants.TURNING_SPEED_PID_CONTROLLER.TOLERANCE);
    }
}
