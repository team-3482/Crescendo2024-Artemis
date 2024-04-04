// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.ElevatorConstants;
import frc.robot.constants.PrimeNumbers;

public class ElevatorSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern
    private static ElevatorSubsystem instance;
    private static Object mutex = new Object();

    public static ElevatorSubsystem getInstance() {
        ElevatorSubsystem result = instance;
		if (result == null) {
			synchronized (mutex) {
				result = instance;
				if (result == null)
					instance = result = new ElevatorSubsystem();
			}
		}
        return instance;
    }

    private CANSparkMax rightClimbMotor = new CANSparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private CANSparkMax leftClimbMotor = new CANSparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);

    /** Creates a new ElevatorSubsystem. Moves the climb mechanism. */
    public ElevatorSubsystem() {
        super("ElevatorSubsystem");

        leftClimbMotor.follow(rightClimbMotor, true);

        setStatusFrames();
    }
    
    /**
     * Sets the speed of the motors to move the climb mechanism
     * @param speed from -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        rightClimbMotor.set(speed);
    }
    
    /**
     * Stops the motors that move the climb mechanism (overloaded)
     */
    public void setSpeed(){
        setSpeed(0);
    }

    @Override
    public void periodic() {}

    private void setStatusFrames() {
        // leftClimbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        leftClimbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, PrimeNumbers.getNextPrimeNumber());
        leftClimbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, PrimeNumbers.getNextPrimeNumber());
        leftClimbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, PrimeNumbers.getNextPrimeNumber());
        leftClimbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, PrimeNumbers.getNextPrimeNumber());
        leftClimbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, PrimeNumbers.getNextPrimeNumber());
        leftClimbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, PrimeNumbers.getNextPrimeNumber());
        // leftClimbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 250);
        
        // rightClimbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        rightClimbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, PrimeNumbers.getNextPrimeNumber());
        rightClimbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, PrimeNumbers.getNextPrimeNumber());
        rightClimbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, PrimeNumbers.getNextPrimeNumber());
        rightClimbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, PrimeNumbers.getNextPrimeNumber());
        rightClimbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, PrimeNumbers.getNextPrimeNumber());
        rightClimbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, PrimeNumbers.getNextPrimeNumber());
        // rightClimbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 250);
    }
}
