// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShootSubsystem extends SubsystemBase {
  private static CANSparkFlex topMotor = new CANSparkFlex(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
  private static CANSparkFlex bottomMotor = new CANSparkFlex(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
  private static CANSparkFlex feederMotor = new CANSparkFlex(ShooterConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);

  private double shooterSpeed, feederSpeed;

  public ShootSubsystem() {
  }

  /**
   * Sets the speed for the motors on the shooter
   * 
   * @param shooterSpeed the speed for the shooter motors.
   * @param feederSpeed  the speed for the feeder motor.
   */
  public void SetShooterMotors(double shooterSpeed, double feederSpeed) {
    this.shooterSpeed = shooterSpeed;
    this.feederSpeed = feederSpeed;
  }

  @Override
  public void periodic() {
    topMotor.set(shooterSpeed);
    bottomMotor.set(-shooterSpeed);
    feederMotor.set(-feederSpeed);

    SmartDashboard.putNumber("Top Motor RPM", topMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Bottom Motor RPM", bottomMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Feeder Motor RPM", feederMotor.getEncoder().getVelocity());
  }
}
