// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.NoteConstants;
import frc.robot.Constants.SwerveKinematics;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.limelight.LimelightSubsystem;
import frc.robot.swerve.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class CenterNoteCommand extends Command {
    private final String LIMELIGHT = LimelightConstants.BACK_LIMELIGHT;

    private final SlewRateLimiter turningLimiter;
    private PIDController pidController;
    private Timer timer;

    /**
    * Creates a new CenterNoteCommand.
    *
    * @param subsystem The subsystem used by this command.
    */
    public CenterNoteCommand() {
        this.turningLimiter = new SlewRateLimiter(NoteConstants.NOTE_TURNING_SLEW_RATE_LIMIT);
        this.pidController = new PIDController(
            NoteConstants.TURNING_SPEED_PID_CONTROLLER.KP,
            NoteConstants.TURNING_SPEED_PID_CONTROLLER.KI,
            NoteConstants.TURNING_SPEED_PID_CONTROLLER.KD);
        this.pidController.setTolerance(NoteConstants.TURNING_SPEED_PID_CONTROLLER.TOLERANCE);
        this.timer = new Timer();

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
        LEDSubsystem.getInstance().setLightState(LightState.SOLID_ORANGE);
        pidController.reset();
        timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        LEDSubsystem.getInstance().setLightState(LightState.SOLID_BLUE);

        double errorDegrees = LimelightSubsystem.getInstance().getHorizontalOffset();

        double turningSpeed = pidController.calculate(Units.degreesToRadians(errorDegrees), 0);
        turningSpeed = turningLimiter.calculate(turningSpeed) * SwerveKinematics.TURNING_SPEED_COEFFIECENT;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, turningSpeed);

        SwerveSubsystem.getInstance().setChassisSpeeds(chassisSpeeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().stopModules();
        this.timer.stop();
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
        return Math.abs(LimelightSubsystem.getInstance().getHorizontalOffset()) <= (NoteConstants.TURNING_SPEED_PID_CONTROLLER.TOLERANCE + 1)
            || timer.get() >= NoteConstants.CENTERING_TIMEOUT;
    }
}
