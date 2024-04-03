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

/** A command that drives the bot forward until there is a note in the sterilizer or it times out. */
public class DriveToNoteCommand extends Command {
    private final SlewRateLimiter driveLimiter;
    private final SlewRateLimiter turningLimiter;
    private PIDController pidController;

    /** Creates a new CenterNoteCommand. */
    public DriveToNoteCommand() {
        setName("DriveToNoteCommand");
        
        this.driveLimiter = new SlewRateLimiter(NoteConstants.NOTE_DRIVE_SLEW_RATE_LIMIT);
        this.turningLimiter = new SlewRateLimiter(NoteConstants.NOTE_TURNING_SLEW_RATE_LIMIT);

        this.pidController = new PIDController(
            NoteConstants.PID.KP,
            NoteConstants.PID.KI,
            NoteConstants.PID.KD);
        this.pidController.setTolerance(NoteConstants.PID.TOLERANCE);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (!LimelightSubsystem.getInstance().hasTarget(LimelightConstants.INTAKE_LLIGHT)) {
            LEDSubsystem.getInstance().setLightState(LightState.WARNING);
            return;
        }
        this.pidController.reset();

        LEDSubsystem.getInstance().setLightState(LightState.AUTO_RUNNING);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        boolean hasTarget = LimelightSubsystem.getInstance().hasTarget(LimelightConstants.INTAKE_LLIGHT);
        double errorDegrees = LimelightSubsystem.getInstance().getHorizontalOffset(LimelightConstants.INTAKE_LLIGHT);
        
        double turningSpeed = this.pidController.calculate(Units.degreesToRadians(errorDegrees), 0);
        turningSpeed = turningLimiter.calculate(turningSpeed) * SwerveKinematics.TURNING_SPEED_COEFFIECENT;
        double drivingSpeed = driveLimiter.calculate(NoteConstants.NOTE_DRIVE_INPUT_SPEED)
            * SwerveKinematics.DRIVE_SPEED_COEFFICENT;
        
        if (!hasTarget) {
            turningSpeed = 0;
            drivingSpeed /= 3;
        }
        
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(drivingSpeed, 0, turningSpeed);
        SwerveSubsystem.getInstance().setChassisSpeeds(chassisSpeeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().stopModules();
        this.pidController.close();
        
        Telemetry.logCommandEnd(getName(), interrupted);
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        Optional<Boolean> secondLaser = SterilizerSubsystem.getInstance().getHasNotes()[1];
        return secondLaser.isPresent() && secondLaser.get();
    }
}
