// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.intake.IntakeSubsystem;

public class IntakePivot extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private PIDController pid;
    private double position;

    public IntakePivot(double position) {
        pid = new PIDController(IntakeConstants.PIVOT_SPEED, 0, 0);
        pid.setTolerance(IntakeConstants.PIVOT_TOLERANCE);
        this.position = position;
        this.addRequirements(IntakeSubsystem.getInstance());  
    }

    @Override
    public void initialize() {
        pid.reset();
    }

    @Override
    public void execute() {
        double speed = pid.calculate(IntakeSubsystem.getInstance().getEncoderPositionRad(), Units.degreesToRadians(position));
        IntakeSubsystem.getInstance().setPivotSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setPivotSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(IntakeSubsystem.getInstance().getEncoderPositionRad() - position) <= IntakeConstants.PIVOT_TOLERANCE;
    }
}
