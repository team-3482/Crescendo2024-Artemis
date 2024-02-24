// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    // Singleton Design Pattern
    private static IntakeSubsystem instance;
    public static IntakeSubsystem getInstance() {
        if(instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    private CANSparkFlex leftMotor = new CANSparkFlex(IntakeConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    private CANSparkFlex rightMotor = new CANSparkFlex(IntakeConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private CANSparkFlex intakeMotor = new CANSparkFlex(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    public IntakeSubsystem() {
        // These motors should be zeroed in the app
        // leftMotor.getEncoder().setPosition(0);
        // rightMotor.getEncoder().setPosition(0);
        
        leftMotor.follow(rightMotor, true);
        rightMotor.getEncoder().setPositionConversionFactor(IntakeConstants.MOTOR_TO_PIVOT_RATIO);
    }

    /**
     * Moves the note forward through the intake
     */
    public void enableIntake() {
        intakeMotor.set(IntakeConstants.INTAKE_SPEED);
    }

    /**
     * Stops the intake motor
     */
    public void stopIntake() {
        intakeMotor.set(0);
    }

    /**
     * Reverses the intake motor
     */
    public void ejectIntake() {
        intakeMotor.set(-IntakeConstants.INTAKE_SPEED);
    }

    /**
     * Set pivot motors to a specific rotation (degrees)
     * 
     * @param speed between -1.0 and 1.0
     */
    public void setPivotSpeed(double speed) {
        rightMotor.set(speed);
    }

    /**
     * Gets the position of the encoder
     * 
     * @return position of the intake in degrees
     */
    public double getPivotPositionDegrees() {
        return Units.rotationsToDegrees(this.rightMotor.getEncoder().getPosition());
    }

    @Override
    public void periodic() {}
}
