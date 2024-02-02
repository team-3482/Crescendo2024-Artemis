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
  private int rgb[];

  public LED(LEDSubsystem subsystem, int[] rgb) {
    m_subsystem = subsystem;
    addRequirements(subsystem);

    this.rgb[0] = rgb[0];
    this.rgb[1] = rgb[1];
    this.rgb[2] = rgb[2];
  }

  @Override
  public void execute() {
    m_subsystem.SetColor(rgb, false);
  }

  @Override
  public void end(boolean i) {
    m_subsystem.SetColor(rgb, true);
  }
}
