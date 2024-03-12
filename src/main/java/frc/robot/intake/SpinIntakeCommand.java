// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import java.util.Optional;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.shooter.SterilizerSubsystem;

/** A command to spin the rollers in the intake at the desired speed. */
public class SpinIntakeCommand extends Command {

    private double speed;
    private double timeout;
    private Timer timer;

    /**
     * Initializes a new IntakeCommand
     * 
     * @param state state of the intake
     * @param timeout the amount of seconds before the command should auto stop
     */
    public SpinIntakeCommand(double speed, double timeout) {
        setName("SpinIntakeCommand");
        this.speed = speed;
        this.timeout = timeout;
        this.timer = new Timer();
    }

    /**
     * Initializes a new IntakeCommand using a default timeout of Double.POSITIVE_INFINITY
     * 
     * @param state state of the intake
     */
    public SpinIntakeCommand(double speed) {
        this(speed, Double.POSITIVE_INFINITY);
    }

    @Override
    public void initialize() {
        LEDSubsystem.getInstance().setLightState(LightState.CMD_INIT);
        this.timer.restart();
        LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);
    }

    @Override
    public void execute() {
        IntakeSubsystem.getInstance().setIntakeSpeed(this.speed);
        
        if (this.speed > 0) {
            SterilizerSubsystem.getInstance().moveForward(false);
        }
        else if (this.speed < 0) {
            SterilizerSubsystem.getInstance().moveBackward(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setIntakeSpeed(0);
        SterilizerSubsystem.getInstance().moveStop();
        this.timer.stop();
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    @Override
    public boolean isFinished() {
        Optional<Boolean> hasNote= SterilizerSubsystem.getInstance().hasNote();
        return (this.timeout != Double.POSITIVE_INFINITY && this.timer.get() >= this.timeout)
            || (hasNote.isPresent() && hasNote.get());
    }
}
