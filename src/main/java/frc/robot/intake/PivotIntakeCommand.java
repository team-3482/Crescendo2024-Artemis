// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;

public class PivotIntakeCommand extends Command {
    private PIDController pid;
    private IntakeState state;

    /**
     * Initializes a new PivotIntakeCommand
     * 
     * @param state of the intake
     */
    public PivotIntakeCommand(IntakeState state) {
        this.state = state;

        this.pid = new PIDController(this.state.getAngle() - IntakeSubsystem.getInstance().getPivotPosition() > 0 ?
            IntakeConstants.PIVOT_PID_P_UP : IntakeConstants.PIVOT_PID_P_DOWN, 0, 0);
        this.pid.setTolerance(IntakeConstants.PIVOT_TOLERANCE);

        this.addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        LEDSubsystem.getInstance().setLightState(LightState.CMD_INIT);
        this.pid.reset();
    }

    @Override
    public void execute() {
        LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);
        double speed = this.pid.calculate(
            Units.degreesToRadians(IntakeSubsystem.getInstance().getPivotPosition()),
            Units.degreesToRadians(this.state.getAngle()));
        IntakeSubsystem.getInstance().setPivotSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setPivotSpeed(0);
        this.pid.close();
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.state.getAngle() - IntakeSubsystem.getInstance().getPivotPosition() ) <= IntakeConstants.PIVOT_TOLERANCE;
    }
}
