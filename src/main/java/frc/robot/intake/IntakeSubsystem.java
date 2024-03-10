// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    // Singleton Design Pattern
    private static IntakeSubsystem instance;

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    private CANSparkFlex leftPivotMotor = new CANSparkFlex(IntakeConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    private CANSparkFlex rightPivotMotor = new CANSparkFlex(IntakeConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    /** Vortex */
    private CANSparkFlex topIntakeMotor = new CANSparkFlex(IntakeConstants.TOP_MOTOR_ID, MotorType.kBrushless);
    /** Neo */
    private CANSparkMax bottomIntakeMotor = new CANSparkMax(IntakeConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);

    public IntakeSubsystem() {
        super("IntakeSubsystem");
        rightPivotMotor.follow(leftPivotMotor, true);
        topIntakeMotor.setInverted(true);
        bottomIntakeMotor.follow(topIntakeMotor, true);
    }

    /**
     * Moves the note forward through the intake
     * 
     * @param intakeSpeed the speed of the intake motors
     */
    public void setIntakeSpeed(double intakeSpeed) {
        topIntakeMotor.set(intakeSpeed);
    }

    /**
     * Set pivot motors to a specific rotation (degrees)
     * 
     * @param speed between -1.0 and 1.0
     */
    public void setPivotSpeed(double speed) {
        leftPivotMotor.set(speed);
    }

    /**
     * Gets the position of the encoder
     * 
     * @return position of the intake in degrees. 0 is at hardware stop when extended.
     */
    public double getPivotPosition() {
        return Units.rotationsToDegrees(this.rightPivotMotor.getEncoder().getPosition() * IntakeConstants.MOTOR_TO_PIVOT_RATIO);
    }

    @Override
    public void periodic() {
        // System.out.println(getPivotPosition());
    }
}
