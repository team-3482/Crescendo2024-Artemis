// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.PhysicalConstants.LimelightConstants;
import frc.robot.constants.Constants.AprilTagConstants;
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
    private ProfiledPIDController ppid;
    /** Used for {@link CenterSpeakerCommand#isFinished()} */
    private double errorRadians;
    /** Used to avoid repeated calls to DS API */
    private Optional<Alliance> alliance;

    public CenterSpeakerCommand() {
        setName("CenterSpeakerCommand");
        
        this.turningLimiter = new SlewRateLimiter(AprilTagConstants.TURNING_SLEW_RATE_LIMIT);
        this.ppid = new ProfiledPIDController(
            AprilTagConstants.TURNING_SPEED_PID_CONTROLLER.KP,
            AprilTagConstants.TURNING_SPEED_PID_CONTROLLER.KI,
            AprilTagConstants.TURNING_SPEED_PID_CONTROLLER.KD, 
            new TrapezoidProfile.Constraints(
                AprilTagConstants.TURNING_SPEED_PID_CONTROLLER.MAX_SPEED,
                AprilTagConstants.TURNING_SPEED_PID_CONTROLLER.MAX_ACCELERATION
            )
        );
        this.ppid.setTolerance(Units.degreesToRadians(AprilTagConstants.TURNING_SPEED_PID_CONTROLLER.TOLERANCE));
        this.ppid.enableContinuousInput(0, 360);
        
        // Adds the swerve subsyetm to requirements to ensure that it is the only class
        // modifying its data at a single time
        // Do not require limelight subsystem because it is getter functions only
        addRequirements(SwerveSubsystem.getInstance()); 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // End the Command if it starts without seeing an AprilTag
        if (!LimelightSubsystem.getInstance().hasTarget(LimelightConstants.SHOOTER_LLIGHT)) {
            CommandScheduler.getInstance().cancel(this);
        }

        this.errorRadians = Units.degreesToRadians(
            LimelightSubsystem.getInstance().getHorizontalOffset(LimelightConstants.SHOOTER_LLIGHT));
        this.ppid.reset(this.errorRadians);
        this.alliance = DriverStation.getAlliance();
        
        LEDSubsystem.getInstance().setLightState(LightState.AUTO_RUNNING);
    }

    @Override
    public void execute() {
        // Skip loops when the LL is not getting proper data, otherwise errorDegrees is 0
        if (LimelightSubsystem.getInstance().getTargetID() != (alliance.get() == Alliance.Red ? 4 : 7)) return;

        this.errorRadians = Units.degreesToRadians(
            LimelightSubsystem.getInstance().getHorizontalOffset(LimelightConstants.SHOOTER_LLIGHT));
        this.errorRadians = this.errorRadians > 0 ? this.errorRadians - Units.degreesToRadians(3) : this.errorRadians;
        
        double turningSpeed = ppid.calculate(this.errorRadians, 0);
        turningSpeed = turningLimiter.calculate(turningSpeed) * AprilTagConstants.TURNING_SPEED_COEFFIECENT;
        
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 
            MathUtil.clamp(turningSpeed, -1.0, 1.0));
        
        SwerveSubsystem.getInstance().setChassisSpeeds(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().stopModules();
        
        Telemetry.logCommandEnd(getName(), interrupted);
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    /**
    * Ends the command when the error is smaller than tolerance at {@link AprilTagConstants}
    */
    @Override
    public boolean isFinished() {
        return Math.abs(this.errorRadians) <= Units.degreesToRadians(AprilTagConstants.TURNING_SPEED_PID_CONTROLLER.TOLERANCE);
    }
}
