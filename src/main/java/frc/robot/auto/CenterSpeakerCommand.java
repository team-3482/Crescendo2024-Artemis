// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.LimelightConstants;
import frc.robot.constants.Constants.AprilTagConstants;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.limelight.LimelightSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utilities.Telemetry;

/**
 * A command that turns the bot until the speaker's AprilTag is centered in front of the shooter.
 * @see {@link LimelightSubsystem} and {@link LimelightConstants#SHOOTER_LLIGHT}.
 */
public class CenterSpeakerCommand extends Command {
    /** Uses radians for calculations. */
    private PIDController turningPID;
    /** Used with end condition in {@link CenterSpeakerCommand#isFinished()}. */
    private double errorRadians;
    /** Check used in {@link CenterSpeakerCommand#end(boolean)} to log if ended early due to seeing no tag. */
    private boolean noTag;

    /**
     * Creates a new CenterSpeakerCommand().
     */
    public CenterSpeakerCommand() {
        setName("CenterSpeakerCommand");

        this.turningPID = new PIDController(
            AprilTagConstants.PID.KP_HIGH,
            AprilTagConstants.PID.KI,
            AprilTagConstants.PID.KD,
            (double) 1 / 30 // This period should somewhat account for LL framerate and latency.
        );
        this.turningPID.setTolerance(Units.degreesToRadians(AprilTagConstants.PID.TOLERANCE));
        this.turningPID.enableContinuousInput(0, 360);
        
        addRequirements(SwerveSubsystem.getInstance()); 
    }

    @Override
    public void initialize() {
        // End the Command if it starts without seeing an AprilTag.
        if (!LimelightSubsystem.getInstance().hasTarget(LimelightConstants.SHOOTER_LLIGHT)) {
            this.noTag = true;
            this.cancel();
            return;
        }
        this.noTag = false;

        this.errorRadians = Units.degreesToRadians(
            LimelightSubsystem.getInstance().getHorizontalOffset(LimelightConstants.SHOOTER_LLIGHT)
        );
        
        // Workaround to initial speeds being too low/high for PID.
        // This is not ideal.
        if (Math.abs(Units.radiansToDegrees(this.errorRadians)) >= 15) {
            this.turningPID.setP(AprilTagConstants.PID.KP_HIGH);
        }
        else if (Math.abs(Units.radiansToDegrees(this.errorRadians)) >= 5) {
            this.turningPID.setP(AprilTagConstants.PID.KP_LOW);
        }
        else {
            this.turningPID.setP(AprilTagConstants.PID.KP_LOWEST);
        }

        this.turningPID.reset();

        LEDSubsystem.getInstance().setLightState(LightState.AUTO_RUNNING);
    }

    @Override
    public void execute() {
        // Skip loops when the LL is not getting the right data, otherwise tx is wrong.
        int tagID = LimelightSubsystem.getInstance().getTargetID();
        if (tagID != 4 && tagID != 7) return;

        this.errorRadians = Units.degreesToRadians(
            LimelightSubsystem.getInstance().getHorizontalOffset(LimelightConstants.SHOOTER_LLIGHT)
        );

        double turningSpeed = this.turningPID.calculate(this.errorRadians, 0) * AprilTagConstants.TURNING_SPEED_COEFFIECENT;

        SwerveSubsystem.getInstance().setChassisSpeeds(
            new ChassisSpeeds(0, 0, turningSpeed)
        );
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().stopModules();
        this.turningPID.close();
        
        Telemetry.logCommandEnd(getName(), interrupted, this.noTag ? "NO TAG SEEN" : "");
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.errorRadians) <= Units.degreesToRadians(AprilTagConstants.PID.TOLERANCE);
    }
}
