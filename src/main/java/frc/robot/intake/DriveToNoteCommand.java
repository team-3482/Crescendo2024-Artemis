// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.NoteConstants;
import frc.robot.Constants.SwerveKinematics;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.limelight.LimelightSubsystem;
import frc.robot.shooter.SterilizerSubsystem;
import frc.robot.swerve.SwerveSubsystem;

/** A command that drives the bot forward until there is a note in the sterilizer or it times out. */
public class DriveToNoteCommand extends Command {
    private final String LIMELIGHT = LimelightConstants.INTAKE_LLIGHT;

    private final SlewRateLimiter driveLimiter;
    private final SlewRateLimiter turningLimiter;
    private PIDController pidController;

    /** Creates a new CenterNoteCommand. */
    public DriveToNoteCommand() {
        setName("DriveToNoteCommand");
        this.driveLimiter = new SlewRateLimiter(NoteConstants.NOTE_DRIVE_SLEW_RATE_LIMIT);
        this.turningLimiter = new SlewRateLimiter(NoteConstants.NOTE_TURNING_SLEW_RATE_LIMIT);

        this.pidController = new PIDController(
            NoteConstants.TURNING_SPEED_PID_CONTROLLER.KP,
            NoteConstants.TURNING_SPEED_PID_CONTROLLER.KI,
            NoteConstants.TURNING_SPEED_PID_CONTROLLER.KD);
        this.pidController.setTolerance(NoteConstants.TURNING_SPEED_PID_CONTROLLER.TOLERANCE);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (!LimelightSubsystem.getInstance().hasTarget(LIMELIGHT)) {
            LEDSubsystem.getInstance().setLightState(LightState.WARNING);
            return;
        }
        LEDSubsystem.getInstance().setLightState(LightState.CMD_INIT);
        pidController.reset();

        LEDSubsystem.getInstance().setLightState(LightState.AUTO_RUNNING);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double errorDegrees = LimelightSubsystem.getInstance().getHorizontalOffset(LimelightConstants.INTAKE_LLIGHT);

        double turningSpeed = pidController.calculate(Units.degreesToRadians(errorDegrees), 0);
        turningSpeed = turningLimiter.calculate(turningSpeed) * SwerveKinematics.TURNING_SPEED_COEFFIECENT;
        double drivingSpeed = driveLimiter.calculate(NoteConstants.NOTE_DRIVE_INPUT_SPEED)
            * SwerveKinematics.DRIVE_SPEED_COEFFICENT;
        
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(drivingSpeed, 0, turningSpeed);
        SwerveSubsystem.getInstance().setChassisSpeeds(chassisSpeeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().stopModules();
        pidController.close();
        if (interrupted) {
            LEDSubsystem.getInstance().setLightState(LightState.WARNING);
        }
        else {
            LEDSubsystem.getInstance().setLightState(LightState.OFF);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        Optional<Boolean> hasNote = SterilizerSubsystem.getInstance().hasNote();
        return (hasNote.isPresent() && hasNote.get());
            // || !LimelightSubsystem.getInstance().hasTarget(LIMELIGHT);
    }
}