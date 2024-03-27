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

/** A command to move the intake to a specific position. */
public class PivotIntakeCommand extends Command {
    private IntakeStates state;
    private PIDController pid;
    private boolean up;

    // private boolean brokenEncoderIsItDown;
    // private Timer brokenEncoderTimer = new Timer();

    /**
     * Initializes a new PivotIntakeCommand
     * @param state of the intake
     */
    public PivotIntakeCommand(IntakeStates state) {
        setName("PivotIntakeCommand");
        this.state = state;
        
        this.up = this.state.getAngle() - IntakeSubsystem.getInstance().getPivotPosition() > 0;
        
        this.pid = new PIDController(
            this.up ? IntakeConstants.PIVOT_PID_P_UP : IntakeConstants.PIVOT_PID_P_DOWN,
            0, 0
        );
        this.pid.setTolerance(this.state.getTolerance());
        this.pid.enableContinuousInput(0, 2 * Math.PI);

        // this.brokenEncoderIsItDown = false;

        addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        this.pid.reset();
        // this.brokenEncoderTimer.restart();
        LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);
    }

    @Override
    public void execute() {
        double speed;
        if (this.up) {
            // this.brokenEncoderIsItDown = false;
            speed = this.pid.calculate(
                Units.degreesToRadians(IntakeSubsystem.getInstance().getPivotPosition()),
                Units.degreesToRadians(this.state.getAngle()));
        }
        else {
            speed = this.pid.calculate(
                Units.degreesToRadians(IntakeSubsystem.getInstance().getPivotPosition()),
                Units.degreesToRadians(this.state.getAngle()));
            // this.brokenEncoderIsItDown = true;
        }
        IntakeSubsystem.getInstance().setPivotSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setPivotSpeed(0);
        this.pid.close();
        
        // if (this.brokenEncoderIsItDown) {
        //     IntakeSubsystem.getInstance().resetPivotPosition(0);
        // }
        // else {
        //     IntakeSubsystem.getInstance().resetPivotPosition(IntakeConstants.IntakeState.IDLE.getAngle());
        // }

        Telemetry.logCommandEnd(getName(), interrupted);
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    @Override
    public boolean isFinished() {
        // return this.brokenEncoderTimer.hasElapsed(1.25);
        return Math.abs(this.state.getAngle() - IntakeSubsystem.getInstance().getPivotPosition()) <= this.state.getTolerance();
    }
}
