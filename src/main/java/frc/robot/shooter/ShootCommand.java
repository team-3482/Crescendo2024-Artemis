// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import java.util.Optional;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SterilizerConstants;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utilities.Telemetry;

/** A command that spins the large wheels of the shooter at the desired speed. */
public class ShootCommand extends Command {
    private boolean reachedRPM;
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
        if (this.state.getCalculateAngle() && !ShooterSubsystem.getInstance().canShoot) {
            end(true);
        }

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

        Optional<Boolean>[] hasNote = SterilizerSubsystem.getInstance().getHasNotes();
        SterilizerSubsystem.getInstance().setSpeed(SterilizerConstants.FEEDING_SPEED);
        
        if (!this.state.getAutoEndShooting()) return;
        
        if (hasNote[0].isEmpty() && hasNote[1].isEmpty()) {
            Timer.delay(1.5);
            end(false);
        }
        else if (!hasNote[0].get() && !hasNote[1].get()) {
            end(false);
        }
        else {
            Timer.delay(0.25);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setShootingVelocities();
        SterilizerSubsystem.getInstance().setSpeed();

        Telemetry.logMessage(this.getName() + (interrupted ? " interrupted" : " ended"), interrupted);
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    /**
     * Always returns false because the command will end itself
     * 
     * @return false
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
