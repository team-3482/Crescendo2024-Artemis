// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class LED extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final LEDSubsystem m_subsystem;

  private int r;
  private int g;
  private int b;

  public LED(LEDSubsystem subsystem, int[] rgb) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    this.r = rgb[0];
    this.g = rgb[1];
    this.b = rgb[2];
  }

  @Override
  public void execute() {
    m_subsystem.SetColor(r, g, b, LEDState.COLOR);
  }

  @Override
  public void end(boolean i) {
    m_subsystem.SetColor(0, 0, 0, LEDState.GRADIENT);
  }
}
