// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants;

public class ClimbSubsystem extends SubsystemBase {
    // Singleton Design Pattern
    private static ClimbSubsystem instance;
    public static ClimbSubsystem getInstance() {
        if(instance == null) {
            instance = new ClimbSubsystem();
        }
        return instance;
    }

    private CANSparkMax leftClimbMotor = new CANSparkMax(PhysicalConstants.ClimbConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    private CANSparkMax rightClimbMotor = new CANSparkMax(PhysicalConstants.ClimbConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private double climbSpeed = 0; 

    /** Creates a new ExampleSubsystem. */
    public ClimbSubsystem() {
        super("ClimbSubsystem");
    }

    public void setClimbSpeed(double speed){
        climbSpeed = speed;
    }

    @Override
    public void periodic() {
        rightClimbMotor.set(climbSpeed);
        leftClimbMotor.follow(rightClimbMotor, true);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
