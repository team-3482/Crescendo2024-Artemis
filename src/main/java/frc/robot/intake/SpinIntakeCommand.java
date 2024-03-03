// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.shooter.SterilizerSubsystem;

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
        this.speed = speed;
        this.timeout = timeout;
        this.timer = new Timer();
    }
    public SpinIntakeCommand(double speed)
    {
        this(speed, Double.POSITIVE_INFINITY);
    }

    @Override
    public void initialize() {
        LEDSubsystem.getInstance().setLightState(LightState.CMD_INIT);
        this.timer.start();
    }

    @Override
    public void execute() {
        LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);
        IntakeSubsystem.getInstance().setIntakeSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().stopIntake();
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    @Override
    public boolean isFinished() {
        return (this.timeout != Double.POSITIVE_INFINITY && this.timer.get() >= this.timeout) || SterilizerSubsystem.getInstance().hasNote().get();
    }
}
