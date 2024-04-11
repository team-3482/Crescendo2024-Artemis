// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.LimelightConstants;
import frc.robot.constants.Constants.NoteConstants;
import frc.robot.constants.PhysicalConstants.SwerveKinematics;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.limelight.LimelightSubsystem;
import frc.robot.shooter.SterilizerSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utilities.Telemetry;

/**
 * A command that drives the bot forward until there is a note in the sterilizer.
 * @see {@link SterilizerSubsystem#getHasNotes()} for the front laser.
 */
public class DriveToNoteCommand extends Command {
    /** Limits the driving acceleration of the robot. */
    private final SlewRateLimiter drivingLimiter;
    /** Limits the turning acceleration of the robot. */
    private final SlewRateLimiter turningLimiter;
    /** Uses radians for calculations. */
    private PIDController turningPID;

    /**
     * Creates a new CenterNoteCommand.
     */
    public DriveToNoteCommand() {
        setName("DriveToNoteCommand");
        
        this.drivingLimiter = new SlewRateLimiter(NoteConstants.NOTE_DRIVE_SLEW_RATE_LIMIT);
        this.turningLimiter = new SlewRateLimiter(NoteConstants.NOTE_TURNING_SLEW_RATE_LIMIT);

        this.turningPID = new PIDController(
            NoteConstants.PID.KP,
            NoteConstants.PID.KI,
            NoteConstants.PID.KD
        );
        this.turningPID.setTolerance(NoteConstants.PID.TOLERANCE);

        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        if (!LimelightSubsystem.getInstance().hasTarget(LimelightConstants.INTAKE_LLIGHT)) {
            LEDSubsystem.getInstance().setLightState(LightState.WARNING);
            return;
        }
        this.turningPID.reset();

        LEDSubsystem.getInstance().setLightState(LightState.AUTO_RUNNING);
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightSubsystem.getInstance().hasTarget(LimelightConstants.INTAKE_LLIGHT);
        double errorDegrees = LimelightSubsystem.getInstance().getHorizontalOffset(LimelightConstants.INTAKE_LLIGHT);
        
        double turningSpeed = this.turningPID.calculate(Units.degreesToRadians(errorDegrees), 0);
        turningSpeed = turningLimiter.calculate(turningSpeed) * SwerveKinematics.TURNING_SPEED_COEFFIECENT;
        
        double drivingSpeed = drivingLimiter.calculate(NoteConstants.NOTE_DRIVE_INPUT_SPEED)
            * SwerveKinematics.DRIVE_SPEED_COEFFICENT;
        
        if (!hasTarget) {
            turningSpeed = 0;
            drivingSpeed /= 3;
        }
        
        SwerveSubsystem.getInstance().setChassisSpeeds(
            new ChassisSpeeds(drivingSpeed, 0, turningSpeed)
        );
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().stopModules();
        this.turningPID.close();
        
        Telemetry.logCommandEnd(getName(), interrupted);
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    @Override
    public boolean isFinished() {
        Optional<Boolean> frontLaser = SterilizerSubsystem.getInstance().getHasNotes()[1];
        return frontLaser.isPresent() && frontLaser.get();
    }
}
