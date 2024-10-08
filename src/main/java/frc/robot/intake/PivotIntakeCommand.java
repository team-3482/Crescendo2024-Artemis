// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.IntakeConstants;
import frc.robot.constants.Constants.IntakeStates;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.utilities.Telemetry;

/**
 * A command to move the intake to a specific position.
 */
public class PivotIntakeCommand extends Command {
    private IntakeStates state;
    /** Uses radians for calculations. */
    private PIDController pid;

    /**
     * Creates a new PivotIntakeCommand.
     * @param state of the intake.
     */
    public PivotIntakeCommand(IntakeStates state) {
        setName("PivotIntakeCommand");
        
        this.state = state;
        
        // P is set in the initialize() method.
        this.pid = new PIDController(0, 0, 0);
        this.pid.setTolerance(this.state.getTolerance());
        this.pid.enableContinuousInput(0, 2 * Math.PI);

        addRequirements(IntakeSubsystem.getInstance().getPivotRequirement());
    }

    @Override
    public void initialize() {
        // Whether or not the intake is moving upwards.
        boolean up = this.state.getAngle() - IntakeSubsystem.getInstance().getPivotPosition() > 0;
        // Uses a different P value on the way up due to acting against gravity.
        this.pid.setP(up ? IntakeConstants.PIVOT_PID_P_UP : IntakeConstants.PIVOT_PID_P_DOWN);
        this.pid.reset();
        
        LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);
    }

    @Override
    public void execute() {
        double speed = this.pid.calculate(
            Units.degreesToRadians(IntakeSubsystem.getInstance().getPivotPosition()),
            Units.degreesToRadians(this.state.getAngle())
        );
        
        IntakeSubsystem.getInstance().setPivotSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setPivotSpeed(0);
        this.pid.close();

        Telemetry.logCommandEnd(getName(), interrupted);
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.state.getAngle() - IntakeSubsystem.getInstance().getPivotPosition()) <= this.state.getTolerance();
    }
}
