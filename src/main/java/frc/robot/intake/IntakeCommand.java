// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;

public class IntakeCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private PIDController pid;
    private double goalPosition;

    /**
     * Initializes a new IntakeCommand
     * 
     * @param goalPosition in degrees
     */
    public IntakeCommand(double goalPosition) {
        pid = new PIDController(IntakeConstants.PIVOT_PID_P, 0, 0);
        pid.setTolerance(IntakeConstants.PIVOT_TOLERANCE);

        this.goalPosition = goalPosition;
        this.addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        LEDSubsystem.getInstance().setLightState(LightState.SOLID_GREEN);
        IntakeSubsystem.getInstance().enableIntake();
        pid.reset();
    }

    @Override
    public void execute() {
        LEDSubsystem.getInstance().setLightState(LightState.SOLID_BLUE);
        double speed = pid.calculate(
            Units.degreesToRadians(IntakeSubsystem.getInstance().getPivotPositionDegrees()),
            Units.degreesToRadians(goalPosition));
        IntakeSubsystem.getInstance().setPivotSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setPivotSpeed(0);
        IntakeSubsystem.getInstance().stopIntake();
        if (interrupted) {
            LEDSubsystem.getInstance().setLightState(LightState.WARNING);
        }
        else {
            LEDSubsystem.getInstance().setLightState(LightState.OFF);
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(IntakeSubsystem.getInstance().getPivotPositionDegrees() - goalPosition) <= IntakeConstants.PIVOT_TOLERANCE;
    }
}
