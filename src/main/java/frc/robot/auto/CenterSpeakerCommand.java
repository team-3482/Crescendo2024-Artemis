// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
    /** Turning {@link PIDController}, uses DEGREES for calculations */
    private PIDController pid;
    /** Used for {@link CenterSpeakerCommand#isFinished()} */
    private double errorRadians;
    private double turningSpeed;
    /** Check to see if ended due to having no tag */
    private boolean noTag;

    public CenterSpeakerCommand() {
        setName("CenterSpeakerCommand");

        this.pid = new PIDController(
            AprilTagConstants.PPID.KP_HIGH,
            AprilTagConstants.PPID.KI,
            AprilTagConstants.PPID.KD,
            (double) 1 / 30
        );
        this.pid.setTolerance(Units.degreesToRadians(AprilTagConstants.PPID.TOLERANCE));
        this.pid.enableContinuousInput(0, 360);
        
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
            this.noTag = true;
            CommandScheduler.getInstance().cancel(this);
            return;
        }
        this.noTag = false;

        this.errorRadians = Units.degreesToRadians(
            LimelightSubsystem.getInstance().getHorizontalOffset(LimelightConstants.SHOOTER_LLIGHT));
        
        if (Math.abs(Units.radiansToDegrees(this.errorRadians)) >= 15) {
            this.pid.setP(AprilTagConstants.PPID.KP_HIGH);
        }
        else if (Math.abs(Units.radiansToDegrees(this.errorRadians)) >= 5) {
            this.pid.setP(AprilTagConstants.PPID.KP_LOW);
        }
        else {
            this.pid.setP(AprilTagConstants.PPID.KP_LOWEST);
        }

        this.pid.reset();
        
        LEDSubsystem.getInstance().setLightState(LightState.AUTO_RUNNING);
    }

    @Override
    public void execute() {
        // Skip loops when the LL is not getting proper data, otherwise errorDegrees is 0
        int tagID = LimelightSubsystem.getInstance().getTargetID();
        if (tagID != 4 && tagID != 7) return;

        this.errorRadians = Units.degreesToRadians(
            LimelightSubsystem.getInstance().getHorizontalOffset(LimelightConstants.SHOOTER_LLIGHT));

        this.turningSpeed = this.pid.calculate(this.errorRadians, 0) * AprilTagConstants.TURNING_SPEED_COEFFIECENT;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, this.turningSpeed);
        SwerveSubsystem.getInstance().setChassisSpeeds(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().stopModules();
        this.pid.close();
        
        Telemetry.logCommandEnd(getName(), interrupted, this.noTag ? "NO TAG SEEN" : "");
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    /**
    * Ends the command when the error is smaller than tolerance at {@link AprilTagConstants}
    */
    @Override
    public boolean isFinished() {
        return Math.abs(this.errorRadians) <= Units.degreesToRadians(AprilTagConstants.PPID.TOLERANCE);
    }
}
