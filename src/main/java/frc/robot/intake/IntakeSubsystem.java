// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
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

    /** Leader for the intake pivot */
    private CANSparkFlex leftPivotMotor = new CANSparkFlex(IntakeConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    /** Follower of {@link IntakeSubsystem#leftPivotMotor} for the intake pivot */
    private CANSparkFlex rightPivotMotor = new CANSparkFlex(IntakeConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

    /** Vortex - leader of {@link IntakeSubsystem#bottomIntakeMotor} for intaking */
    private CANSparkFlex topIntakeMotor = new CANSparkFlex(IntakeConstants.TOP_MOTOR_ID, MotorType.kBrushless);
    /** Neo - Follower for intaking*/
    private CANSparkMax bottomIntakeMotor = new CANSparkMax(IntakeConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
    /** Through bore encoder in absolute mode */
    private SparkAbsoluteEncoder pivotEncoder = bottomIntakeMotor.getAbsoluteEncoder();

    public IntakeSubsystem() {
        super("IntakeSubsystem");

        leftPivotMotor.setInverted(false);
        rightPivotMotor.follow(leftPivotMotor, true);

        topIntakeMotor.setInverted(true);
        bottomIntakeMotor.follow(topIntakeMotor, true);
        
        pivotEncoder.setInverted(true);  // Inverted so the IntakeState.IDLE position is positive
    }

    /**
     * Sets the speeds of the top and bottom intaking motors.
     * 
     * @param speed of the intake motors between -1.0 and 1.0
     */
    public void setIntakeSpeed(double speed) {
        topIntakeMotor.set(speed);
    }

    /**
     * Set pivot motors to a specific speed when position is within the bounds provided by
     * {@link IntakeState#INTAKING} and {@link IntakeState#IDLE}
     * 
     * @param speed between -1.0 and 1.0
     * @param safe stop when at the soft stops
     */
    public void setPivotSpeed(double speed, boolean safe) {
        if (safe) {
            double position = getPivotPosition();
            speed = 
                (speed < 0 && Math.abs(IntakeState.INTAKING.getAngle() - position) <= IntakeState.INTAKING.getTolerance()) ||
                (speed > 0 && Math.abs(IntakeState.IDLE.getAngle() - position) <= IntakeState.IDLE.getTolerance())
                    ? 0 : speed;
        }
        leftPivotMotor.set(speed);
    }

    /**
     * Set pivot motors to a specific speed safely (overloaded)
     * 
     * @param speed between -1.0 and 1.0
     */
    public void setPivotSpeed(double speed) {
        setPivotSpeed(speed, true);
    }

    /**
     * Gets the absolute position of the through bore encoder.
     * 
     * @return position of the intake in degrees.
     *  
     * @apiNote 0 is at hard stop when extended.
     */
    public double getPivotPosition() {
        double position = Units.rotationsToDegrees(this.pivotEncoder.getPosition());
        // Because it's an absolute encoder, make sure it isn't returning values like 359 or 358
        if (position > 350)
            position = 0;
        return position;
    }

    @Override
    public void periodic() {}
}
