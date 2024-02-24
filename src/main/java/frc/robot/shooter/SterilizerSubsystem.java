// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import java.util.Optional;

import com.revrobotics.CANSparkMax;

import au.grapplerobotics.LaserCan;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SterilizerConstants;

public class SterilizerSubsystem extends SubsystemBase {
    // Singleton Design Pattern
    private static SterilizerSubsystem instance;
    public static SterilizerSubsystem getInstance() {
        if(instance == null) {
            instance = new SterilizerSubsystem();
        }
        return instance;
    }

    private CANSparkMax neoMotor = new CANSparkMax(SterilizerConstants.NEO_MOTOR_ID, MotorType.kBrushless);
    private LaserCan laser = new LaserCan(SterilizerConstants.LASER_ID);

    /** Creates a new ShooterSubsystem.*/
    public SterilizerSubsystem() {
        // LaserCAN configured in GrappleHook app
    }
    
    /**
     * Returns whether or not there is a note in the subsystem (laser broken)
     * 
     * @return contains a note, null if invalid measurements
     */
    public Optional<Boolean> hasNote() {
        LaserCan.Measurement measurement = laser.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return Optional.ofNullable(measurement.distance_mm <= SterilizerConstants.NOTE_DISTANCE_LASER);
        }
        return null;
    }
    
    /**
     * Moves the note forwards to the shooter
     */
    public void moveForward() {
        neoMotor.set(SterilizerConstants.MOVING_SPEED);
    }
    
    /**
     * Moves the note backwards to the intake
     */
    public void moveBackward() {
        neoMotor.set(-SterilizerConstants.MOVING_SPEED);
    }
    
    /**
     * Stops the NEO motor
     */
    public void stopMoving() {
        neoMotor.set(0);
    }

    @Override
    public void periodic() {}
}
