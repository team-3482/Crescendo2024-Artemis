// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private static CANSparkFlex leftMotor = new CANSparkFlex(IntakeConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private static CANSparkFlex rightMotor = new CANSparkFlex(IntakeConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
  //private static CANSparkFlex intakeMotor = new CANSparkFlex(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

  private double intakeSpeed;

  public IntakeSubsystem() {
    leftMotor.getEncoder().setPosition(0);
    rightMotor.getEncoder().setPosition(0);
  }

  /**
   * Sets the speed for the intake motor
   * 
   * @param intakeSpeed the speed for the intake motor.
   */
  public void SetIntakeMotor(double intakeSpeed) {
    this.intakeSpeed = intakeSpeed;
  }

  /**
   * Set pivot motors to a specific rotation (degrees)
   * 
   * @param speed the rotation speed to move with
   */
  public void SetPivotSpeed(double speed) {
    leftMotor.set(-speed);
    rightMotor.set(speed);
  }

  public double getEncoderPositionRad() {
    return Units.rotationsToRadians(this.rightMotor.getEncoder().getPosition());
  }
  @Override
  public void periodic() {
    //intakeMotor.set(intakeSpeed);

    //SmartDashboard.putNumber("Intake Motor RPM", intakeMotor.getEncoder().getVelocity());
  }
}
