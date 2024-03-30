// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants.NoteConstants;
import frc.robot.constants.Constants.AprilTagConstants;
import frc.robot.constants.PhysicalConstants.LimelightConstants;
import frc.robot.constants.PhysicalConstants.SwerveKinematics;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.limelight.LimelightSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utilities.Telemetry;

/** An command to turn the bot until the note it sees is centered in front of the intake. */
public class CenterNoteCommand extends Command {
    private final String LIMELIGHT = LimelightConstants.INTAKE_LLIGHT;

    private final SlewRateLimiter turningLimiter;
    private PIDController pid;
    /** Used for {@link CenterNoteCommand#isFinished()} */
    private double errorRadians;

    /**
    * Creates a new CenterNoteCommand.
    */
    public CenterNoteCommand() {
        setName("CenterNoteCommand");
        
        this.turningLimiter = new SlewRateLimiter(NoteConstants.NOTE_TURNING_SLEW_RATE_LIMIT);
        this.pid = new PIDController(
            NoteConstants.TURNING_SPEED_PID_CONTROLLER.KP,
            NoteConstants.TURNING_SPEED_PID_CONTROLLER.KI,
            NoteConstants.TURNING_SPEED_PID_CONTROLLER.KD);
        this.pid.setTolerance(Units.degreesToRadians(NoteConstants.TURNING_SPEED_PID_CONTROLLER.TOLERANCE));

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // End the Command if it starts without seeing a note
        if (!LimelightSubsystem.getInstance().hasTarget(LIMELIGHT)) {
            CommandScheduler.getInstance().cancel(this);
        }
        this.errorRadians = AprilTagConstants.TURNING_SPEED_PID_CONTROLLER.TOLERANCE + 1;
        this.pid.reset();
        
        LEDSubsystem.getInstance().setLightState(LightState.AUTO_RUNNING);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Skip loops when the LL is not getting proper data, otherwise errorDegrees is 0
        if (!LimelightSubsystem.getInstance().hasTarget(LimelightConstants.INTAKE_LLIGHT)) return;

        this.errorRadians = Units.degreesToRadians(
            LimelightSubsystem.getInstance().getHorizontalOffset(LimelightConstants.INTAKE_LLIGHT));

        double turningSpeed = pid.calculate(this.errorRadians, 0);
        turningSpeed = turningLimiter.calculate(turningSpeed) * SwerveKinematics.TURNING_SPEED_COEFFIECENT;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, turningSpeed);

        SwerveSubsystem.getInstance().setChassisSpeeds(chassisSpeeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().stopModules();
        this.pid.close();
        
        Telemetry.logCommandEnd(getName(), interrupted);
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.errorRadians <= Units.degreesToRadians(NoteConstants.TURNING_SPEED_PID_CONTROLLER.TOLERANCE);
    }
}
