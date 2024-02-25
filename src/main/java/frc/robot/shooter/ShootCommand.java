// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import java.util.Optional;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;

/** An example command that uses an example subsystem. */
public class ShootCommand extends Command {
    private double[] goals;
    private boolean finished;

    /**
    * Creates a new ShootCommand.
    *
    * @param subsystem The subsystem used by this command.
    */
    public ShootCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem.getInstance());
    }

    private double[] calculateShootingSpeedAndPosition(double distance) {
        // Get interpolated values from table
        return new double[3];
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        LEDSubsystem.getInstance().setLightState(LightState.CMD_INIT);
        this.finished = false;
        this.goals = calculateShootingSpeedAndPosition(0);
        ShooterSubsystem.getInstance().setPivotPosition(this.goals[2]);
        ShooterSubsystem.getInstance().setShootingVelocities(new double[]{
            goals[0], goals[1]
        });
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);

        double position = ShooterSubsystem.getInstance().getPivotPosition();
        if (Math.abs(this.goals[2] - position) > ShooterConstants.ALLOWED_PIVOT_ERROR) return;
        
        double[] velocities = ShooterSubsystem.getInstance().getShootingVelocities();
        if (Units.radiansPerSecondToRotationsPerMinute(this.goals[0] - velocities[0]) > ShooterConstants.ALLOWED_RPM_ERROR
            || Units.radiansPerSecondToRotationsPerMinute(this.goals[1] - velocities[1]) > ShooterConstants.ALLOWED_RPM_ERROR
        ) return;

        Optional<Boolean> hasNote = SterilizerSubsystem.getInstance().hasNote(); 
        if (!hasNote.isPresent() && hasNote.get()) {
            SterilizerSubsystem.getInstance().moveForward();
        }
        Timer.delay(1);
        this.finished = true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setShootingVelocities(new double[]{0, 0});
        ShooterSubsystem.getInstance().setPivotPosition(0);
        SterilizerSubsystem.getInstance().stopMoving();

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
        return this.finished;
    }
}
