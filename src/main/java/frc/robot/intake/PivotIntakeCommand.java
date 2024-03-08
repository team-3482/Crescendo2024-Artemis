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
    private PIDController pivotPIDController;
    private IntakeState state;

    /**
     * Initializes a new PivotIntakeCommand
     * 
     * @param state of the intake
     */
    public PivotIntakeCommand(IntakeState state) {
        this.pivotPIDController = new PIDController(IntakeConstants.PIVOT_PID_P, 0, 0);
        this.pivotPIDController.setTolerance(IntakeConstants.PIVOT_TOLERANCE);

        this.state = state;
        this.addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        LEDSubsystem.getInstance().setLightState(LightState.CMD_INIT);
        this.pivotPIDController.reset();
    }

    @Override
    public void execute() {
        LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);
        double speed = this.pivotPIDController.calculate(
            Units.degreesToRadians(IntakeSubsystem.getInstance().getPivotPositionDegrees()),
            Units.degreesToRadians(this.state.getAngle()));
        IntakeSubsystem.getInstance().setPivotSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setPivotSpeed(0);
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(IntakeSubsystem.getInstance().getPivotPositionDegrees() - this.state.getAngle()) <= IntakeConstants.PIVOT_TOLERANCE;
    }
}
