// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import java.util.Optional;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ShooterStates;
import frc.robot.constants.PhysicalConstants.SterilizerConstants;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utilities.Telemetry;

/**
 * A command that spins the shooting wheels to a desired speed and
 * feeds a note to them.
 */
public class ShootCommand extends Command {
    private ShooterStates state;
    private boolean invertSpin;
    /** Move the sterilizer once speeds are within error and then ignore the speeds entirely. */
    private boolean reachedRPM;
    private boolean finished;

    /**
     * Creates a new ShootCommand.
     * @param state the shooter state to follow.
     */
    public ShootCommand(ShooterStates state) {
        setName("ShootCommand");

        this.state = state;
        
        addRequirements(ShooterSubsystem.getInstance().getShootingRequirement(), SterilizerSubsystem.getInstance());
    }
    
    @Override
    public void initialize() {
        this.finished = false;

        this.invertSpin = !this.state.getCalculateAngle()
            || SwerveSubsystem.getInstance().getHeading() < 180 ? false : true;
        ShooterSubsystem.getInstance().setShootingVelocities(this.state.getRPMs(this.invertSpin));

        this.reachedRPM = false;

        LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);
    }

    @Override
    public void execute() {
        if (!this.reachedRPM) {
            double[] velocities = ShooterSubsystem.getInstance().getShootingVelocities();
            double[] rpmGoals = this.state.getRPMs(this.invertSpin);
            System.out.println("Diff : [" + (int) Math.abs(rpmGoals[0] - velocities[0])
                + " | " + (int) Math.abs(rpmGoals[1] - velocities[1]) +  "]"
            );
            
            if (Math.abs(rpmGoals[0] - velocities[0]) > this.state.getAllowedError()
                || Math.abs(rpmGoals[1] - velocities[1]) > this.state.getAllowedError())
                return;
            this.reachedRPM = true;
        }

        Optional<Boolean>[] hasNote = SterilizerSubsystem.getInstance().getHasNotes();
        SterilizerSubsystem.getInstance().setSpeed(SterilizerConstants.FEEDING_SPEED);
        
        if (!this.state.getAutoEndShooting()) return;
        
        if (hasNote[0].isEmpty() || hasNote[1].isEmpty()) {
            Timer.delay(1);
            this.finished = true;
        }
        else if (!hasNote[0].get() && !hasNote[1].get()) {
            this.finished = true;
        }
        else {
            Timer.delay(0.25);
        }
    }

    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setShootingVelocities();
        SterilizerSubsystem.getInstance().setSpeed();

        Telemetry.logCommandEnd(getName(), interrupted);
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    @Override
    public boolean isFinished() {
        return this.finished;
    }
}
