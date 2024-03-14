// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.swerve.SwerveSubsystem;

/** A command that spins the large wheels of the shooter at half of the desired speed. */
public class RevUpCommand extends Command {
    private ShooterState state;
    private boolean invertSpin;

    /**
    * Creates a new RevUpCommand.
    *
    * @param state the shooter state to follow
    */
    public RevUpCommand(ShooterState state) {
        setName("RevUpCommand");
        this.state = state;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem.getInstance());
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        LEDSubsystem.getInstance().setLightState(LightState.CMD_INIT);
        if (this.state.calculateAngle() && !ShooterSubsystem.getInstance().canShoot) {
            end(true);
        }

        this.invertSpin = !this.state.calculateAngle()
            || SwerveSubsystem.getInstance().getHeading() < 180 ?
                false : true;
        double[] speeds = this.state.getSpeeds(this.invertSpin);
        ShooterSubsystem.getInstance().setShootingVelocities(
            new double[]{speeds[0] / 2, speeds[1] / 2});

        LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        Optional<Boolean> hasNote = SterilizerSubsystem.getInstance().hasNote();
        return hasNote.isPresent() && hasNote.get();
    }
}
