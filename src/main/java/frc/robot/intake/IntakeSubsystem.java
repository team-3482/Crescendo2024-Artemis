// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;


import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;

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
    /** Through bore encoder */
    private RelativeEncoder pivotEncoder = bottomIntakeMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

    public IntakeSubsystem() {
        super("IntakeSubsystem");
        leftPivotMotor.setInverted(false);
        rightPivotMotor.follow(leftPivotMotor, true);
        topIntakeMotor.setInverted(true);
        bottomIntakeMotor.follow(topIntakeMotor, true);
        pivotEncoder.setInverted(true);

        resetPivotPosition(IntakeConstants.IntakeState.IDLE.getAngle());
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
     * Set pivot motors to a specific speed
     * 
     * @param speed between -1.0 and 1.0
     */
    public void setPivotSpeed(double speed) {
        leftPivotMotor.set(speed);
    }

    /**
     * Set pivot motors to a specific speed but stops near extremities
     * 
     * @param speed between -1.0 and 1.0
     */
    public void setPivotSpeedSafe(double speed) {
        double position = getPivotPosition();
        speed = 
            (speed < 0 && Math.abs(IntakeState.INTAKING.getAngle() - position) <= IntakeState.INTAKING.getTolerance()) ||
            (speed > 0 && Math.abs(IntakeState.IDLE.getAngle() - position) <= IntakeState.IDLE.getTolerance())
                ? 0 : speed;

        leftPivotMotor.set(speed);
    }

    /**
     * Gets the position of the through bore encoder
     * 
     * @return position of the intake in degrees. 0 is at hard stop when extended.
     */
    public double getPivotPosition() {
        return Units.rotationsToDegrees(this.pivotEncoder.getPosition());
    }

    /**
     * Sets the position of the through bore encoder
     * 
     * @return position of the intake in degrees. 0 is at hard stop when extended.
     */
    public void resetPivotPosition(double position) {
        this.pivotEncoder.setPosition(Units.degreesToRotations(position));
    }

    @Override
    public void periodic() {
        int position = (int) getPivotPosition();
        if (this.pivotEncoder.getVelocity() == 0 && position <= 10) {
            resetPivotPosition(0);
        }
    }
}
