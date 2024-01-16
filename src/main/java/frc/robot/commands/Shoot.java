// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ShootSubsystem;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  private static CANSparkFlex leftMotor = new CANSparkFlex(IntakeConstants.leftShooterMotorId, MotorType.kBrushless);
  private static CANSparkFlex rightMotor = new CANSparkFlex(IntakeConstants.rightShooterMotorId, MotorType.kBrushless);

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShootSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot(ShootSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftMotor.set(-IntakeConstants.shooterSpeed);
    rightMotor.set(IntakeConstants.shooterSpeed);

    System.out.println(leftMotor.getEncoder());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
