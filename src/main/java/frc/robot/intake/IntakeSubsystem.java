// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.IntakeConstants;
import frc.robot.constants.PrimeNumbers;
import frc.robot.constants.Constants.IntakeStates;

public class IntakeSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern
    private static volatile IntakeSubsystem instance;
    private static Object mutex = new Object();

    public static IntakeSubsystem getInstance() {
        IntakeSubsystem result = instance;

        if (result == null) {
			synchronized (mutex) {
				result = instance;
				if (result == null) {
					instance = result = new IntakeSubsystem();
                }
            }
        }
        return instance;
    }

    private class PivotRequirement extends SubsystemBase {
        public PivotRequirement() {
            setName("Intake - Pivot Requirement");
        }
    }

    private class IntakingRequirement extends SubsystemBase {
        public IntakingRequirement() {
            setName("Intake - Intaking Requirement");
        }
    }

    // private SubsystemBase pivotRequirement = new SubsystemBase("Intake - Pivot Requirement") {};
    private PivotRequirement pivotRequirement = new PivotRequirement();
    // private SubsystemBase intakingRequirement = new SubsystemBase("Intake - Intaking Requirement") {};
    private IntakingRequirement intakingRequirement = new IntakingRequirement();

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

        setStatusFrames();
    }

    /**
     * Sets the speeds of the top and bottom intaking motors.
     * @param speed of the intake motors between -1.0 and 1.0
     */
    public void setIntakeSpeed(double speed) {
        topIntakeMotor.set(speed);
    }

    /**
     * Sets the speeds of the top and bottom intaking motors to 0. (overloaded)
     */
    public void setIntakeSpeed() {
        setIntakeSpeed(0);
    }

    /**
     * Set pivot motors to a specific speed when position is within the bounds provided by
     * {@link IntakeStates#INTAKING} and {@link IntakeStates#IDLE}
     * @param speed between -1.0 and 1.0
     * @param safe stop when at the soft stops
     */
    public void setPivotSpeed(double speed, boolean safe) {
        if (safe) {
            double position = getPivotPosition();
            speed = 
                (speed < 0 && Math.abs(IntakeStates.INTAKING.getAngle() - position) <= 1) ||
                (speed > 0 && IntakeStates.IDLE.getAngle() - position <= 1)
                    ? 0 : speed;
        }
        leftPivotMotor.set(speed);
    }

    /**
     * Set pivot motors to a specific speed safely (overloaded)
     * @param speed between -1.0 and 1.0
     */
    public void setPivotSpeed(double speed) {
        setPivotSpeed(speed, true);
    }

    /**
     * Set pivot motors to 0 (overloaded)
     */
    public void setPivotSpeed() {
        setPivotSpeed(0);
    }

    /**
     * Gets the absolute position of the through bore encoder.
     * @return position of the intake in degrees.
     * @apiNote 0 is at hard stop when extended.
     */
    public double getPivotPosition() {
        double position = Units.rotationsToDegrees(this.pivotEncoder.getPosition());
        // Because it's an absolute encoder, make sure it isn't returning values like 359 or 358
        if (position > 350)
            position = 0;
        return position;
    }

    /**
     * Returns a subsystem to be used with Command requirements
     * @return pivot subsystem
     */
    public Subsystem getPivotRequirement() {
        return this.pivotRequirement;
    }

    /**
     * Returns a subsystem to be used with Command requirements
     * @return intaking subsystem
     */
    public Subsystem getIntakingRequirement() {
        return this.intakingRequirement;
    }

    @Override
    public void periodic() {}

    /**
     * Limits the publishing of CAN messages to the bus that we do not use
     */
    private void setStatusFrames() {
        // leftPivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        leftPivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, PrimeNumbers.getNextPrimeNumber());
        leftPivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, PrimeNumbers.getNextPrimeNumber());
        // leftPivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
        leftPivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, PrimeNumbers.getNextPrimeNumber());
        leftPivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, PrimeNumbers.getNextPrimeNumber());
        leftPivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, PrimeNumbers.getNextPrimeNumber());
        // leftPivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 250);

        // rightPivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        rightPivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, PrimeNumbers.getNextPrimeNumber());
        rightPivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, PrimeNumbers.getNextPrimeNumber());
        // rightPivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
        rightPivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, PrimeNumbers.getNextPrimeNumber());
        rightPivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, PrimeNumbers.getNextPrimeNumber());
        rightPivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, PrimeNumbers.getNextPrimeNumber());
        // rightPivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 250);

        // topIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        topIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, PrimeNumbers.getNextPrimeNumber());
        topIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, PrimeNumbers.getNextPrimeNumber());
        // topIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
        // topIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, PrimeNumbers.getNextPrimeNumber());
        // topIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, PrimeNumbers.getNextPrimeNumber());
        topIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, PrimeNumbers.getNextPrimeNumber());
        // topIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 250);

        // bottomIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        bottomIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, PrimeNumbers.getNextPrimeNumber());
        bottomIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, PrimeNumbers.getNextPrimeNumber());
        // bottomIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
        // bottomIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 20);
        // bottomIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, PrimeNumbers.getNextPrimeNumber());
        bottomIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, PrimeNumbers.getNextPrimeNumber());
        // bottomIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 250);
    }
}
