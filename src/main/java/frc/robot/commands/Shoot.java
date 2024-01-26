// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ShootSubsystem;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  private static CANSparkFlex topMotor = new CANSparkFlex(IntakeConstants.topShooterMotorId, MotorType.kBrushless);
  private static CANSparkFlex bottomMotor = new CANSparkFlex(IntakeConstants.bottomShooterMotorId,
      MotorType.kBrushless);
  private static CANSparkFlex feederMotor = new CANSparkFlex(IntakeConstants.feederShooterMotorId,
      MotorType.kBrushless);

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
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
    topMotor.set(IntakeConstants.shooterSpeed);
    bottomMotor.set(-IntakeConstants.shooterSpeed);
    feederMotor.set(IntakeConstants.feederSpeed);

    SmartDashboard.putNumber("Top Motor RPM", topMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Bottom Motor RPM", bottomMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Feeder Motor RPM", feederMotor.getEncoder().getVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    topMotor.set(0);
    bottomMotor.set(0);
    feederMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
