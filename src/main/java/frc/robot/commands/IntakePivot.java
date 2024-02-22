// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePivot extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IntakeSubsystem m_subsystem;

  public IntakePivot(IntakeSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_subsystem.SetPivot(IntakeConstants.PIVOT_DOWN_DEGREE);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.SetPivot(IntakeConstants.PIVOT_UP_DEGREE);
  }
}
