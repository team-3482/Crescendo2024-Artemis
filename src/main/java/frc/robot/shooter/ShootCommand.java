// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import java.text.DecimalFormat;
import java.util.Optional;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.swerve.SwerveSubsystem;

/** A command that spins the large wheels of the shooter at the desired speed. */
public class ShootCommand extends Command {
    private boolean reachedRPM;
    private boolean finished;
    private ShooterState state;
    private boolean invertSpin;

    /**
    * Creates a new ShootCommand.
    *
    * @param state the shooter state to follow
    */
    public ShootCommand(ShooterState state) {
        setName("ShootCommand");
        this.state = state;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem.getInstance(), SterilizerSubsystem.getInstance());
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        LEDSubsystem.getInstance().setLightState(LightState.CMD_INIT);
        if (this.state.getCalculateAngle() && !ShooterSubsystem.getInstance().canShoot) {
            end(true);
        }
        this.finished = false;

        this.invertSpin = !this.state.getCalculateAngle()
            || SwerveSubsystem.getInstance().getHeading() < 180 ?
                false : true;
        ShooterSubsystem.getInstance().setShootingVelocities(this.state.getRPMs(this.invertSpin));

        this.reachedRPM = false;

        LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double[] velocities = ShooterSubsystem.getInstance().getShootingVelocities();
        double[] rpmGoals = this.state.getRPMs(this.invertSpin);
        System.out.println("Diff : [" + (int) Math.abs(rpmGoals[0] - velocities[0]) + " | " + (int) Math.abs(rpmGoals[1] - velocities[1]) +  "]");
        
        if (!this.reachedRPM
            && ((Math.abs(rpmGoals[0] - velocities[0]) > this.state.getAllowedError()
            || Math.abs(rpmGoals[1] - velocities[1]) > this.state.getAllowedError())))
            return;
        this.reachedRPM = true;

        Optional<Boolean> hasNote = SterilizerSubsystem.getInstance().hasNoteMeasurement();
        SterilizerSubsystem.getInstance().moveForward(false);
        if (!this.state.getAutoEndShooting()) return;
        if (!hasNote.isPresent()) {
            Timer.delay(2.5);
            this.finished = true;
        } 
        else if (hasNote.get()) {
            Timer.delay(0.5);
        }
        if (hasNote.isPresent() && !hasNote.get()) {
            this.finished = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setShootingVelocities(new double[]{0, 0});
        SterilizerSubsystem.getInstance().moveStop();

        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.state.getAutoEndShooting() ? this.finished : false;
    }
}
