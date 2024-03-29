// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants;

public class ElevatorSubsystem extends SubsystemBase {
    // Singleton Design Pattern
    private static ElevatorSubsystem instance;
    public static ElevatorSubsystem getInstance() {
        if(instance == null) {
            instance = new ElevatorSubsystem();
        }
        return instance;
    }

    private CANSparkMax rightClimbMotor = new CANSparkMax(PhysicalConstants.ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private CANSparkMax leftClimbMotor = new CANSparkMax(PhysicalConstants.ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);

    /** Creates a new ElevatorSubsystem. Moves the climb mechanism. */
    public ElevatorSubsystem() {
        super("ElevatorSubsystem");

        leftClimbMotor.follow(rightClimbMotor, true);
    }
    
    /**
     * Sets the speed of the motors to move the climb mechanism
     * @param speed from -1.0 to 1.0
     */
    public void setSpeed(double speed){
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
}
