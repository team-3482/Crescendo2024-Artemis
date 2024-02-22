// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private static CANSparkFlex leftMotor = new CANSparkFlex(IntakeConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private static CANSparkFlex rightMotor = new CANSparkFlex(IntakeConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
  private static CANSparkFlex intakeMotor = new CANSparkFlex(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

  private double intakeSpeed;

  public IntakeSubsystem() {
  }

  /**
   * Sets the speed for the motors on the shooter
   * 
   * @param intakeSpeed the speed for the intake motor.
   */
  public void SetIntakeMotor(double intakeSpeed) {
    this.intakeSpeed = intakeSpeed;
  }

  public void IntakePivot(int degree) {

  }

  @Override
  public void periodic() {
    intakeMotor.set(intakeSpeed);

    SmartDashboard.putNumber("Intake Motor RPM", intakeMotor.getEncoder().getVelocity());
  }
}
