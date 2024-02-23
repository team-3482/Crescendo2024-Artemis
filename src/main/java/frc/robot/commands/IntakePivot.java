// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePivot extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IntakeSubsystem m_subsystem;
  private PIDController pid;
  private double position;

  public IntakePivot(IntakeSubsystem subsystem, double position) {
    pid = new PIDController(IntakeConstants.PIVOT_SPEED, 0, 0);
    pid.setTolerance(IntakeConstants.PIVOT_TOLERANCE);
    this.position = position;
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    pid.reset();
  }

  @Override
  public void execute() {
    double speed = pid.calculate(m_subsystem.getEncoderPositionRad(), Units.degreesToRadians(position));
    m_subsystem.SetPivotSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("ended");
    m_subsystem.SetPivotSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_subsystem.getEncoderPositionRad() - position)
      <= IntakeConstants.PIVOT_TOLERANCE;
  }
}
