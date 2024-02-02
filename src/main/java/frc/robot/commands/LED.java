// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class LED extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final LEDSubsystem m_subsystem;

  private int r;
  private int g;
  private int b;

  public LED(LEDSubsystem subsystem, int r, int g, int b) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    this.r = r;
    this.g = g;
    this.b = b;

  }

  @Override
  public void execute() {
    m_subsystem.SetColor(r, g, b, false);
  }

  @Override
  public void end(boolean i) {
    m_subsystem.SetColor(0, 0, 0, true);
  }
}
