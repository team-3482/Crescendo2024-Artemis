// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.NoteConstants;
import frc.robot.constants.PhysicalConstants.LimelightConstants;
import frc.robot.constants.PhysicalConstants.SwerveKinematics;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.limelight.LimelightSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utilities.Telemetry;

/**
 * A command that turns the bot until the note it sees is centered in front of the intake.
 * @see {@link LimelightSubsystem} and {@link LimelightConstants#INTAKE_LLIGHT}.
 */
public class CenterNoteCommand extends Command {
    /** Limits the turning acceleration of the robot. */
    private SlewRateLimiter turningLimiter;
    /** Uses radians for calculations. */
    private PIDController turningPID;
    /** Used with end condition in {@link CenterNoteCommand#isFinished()}. */
    private double errorRadians;

    /**
     * Creates a new CenterNoteCommand.
     */
    public CenterNoteCommand() {
        setName("CenterNoteCommand");
        
        this.turningLimiter = new SlewRateLimiter(NoteConstants.NOTE_TURNING_SLEW_RATE_LIMIT);
        this.turningPID = new PIDController(
            NoteConstants.PID.KP,
            NoteConstants.PID.KI,
            NoteConstants.PID.KD
        );
        this.turningPID.setTolerance(Units.degreesToRadians(NoteConstants.PID.TOLERANCE));

        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        // End the Command if it starts without seeing a note.
        if (!LimelightSubsystem.getInstance().hasTarget(LimelightConstants.INTAKE_LLIGHT)) {
            this.cancel();
        }

        // So the command doesn't end immediately in case the Limelight loses its target.
        this.errorRadians = NoteConstants.PID.TOLERANCE + 1;
        this.turningPID.reset();
        
        LEDSubsystem.getInstance().setLightState(LightState.AUTO_RUNNING);
    }

    @Override
    public void execute() {
        // Skip loops when the LL is not getting proper data, otherwise tx is 0.
        if (!LimelightSubsystem.getInstance().hasTarget(LimelightConstants.INTAKE_LLIGHT)) return;

        this.errorRadians = Units.degreesToRadians(
            LimelightSubsystem.getInstance().getHorizontalOffset(LimelightConstants.INTAKE_LLIGHT)
        );

        double turningSpeed = turningPID.calculate(this.errorRadians, 0);
        turningSpeed = turningLimiter.calculate(turningSpeed) * SwerveKinematics.TURNING_SPEED_COEFFIECENT;

        SwerveSubsystem.getInstance().setChassisSpeeds(
            new ChassisSpeeds(0, 0, turningSpeed)
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
        return Math.abs(this.errorRadians) <= Units.degreesToRadians(NoteConstants.PID.TOLERANCE);
    }
}
