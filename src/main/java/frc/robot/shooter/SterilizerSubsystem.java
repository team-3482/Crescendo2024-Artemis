// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import java.util.Optional;

import com.revrobotics.CANSparkFlex;

import au.grapplerobotics.LaserCan;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SterilizerConstants;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;

public class SterilizerSubsystem extends SubsystemBase {
    // Singleton Design Pattern
    private static SterilizerSubsystem instance;
    public static SterilizerSubsystem getInstance() {
        if(instance == null) {
            instance = new SterilizerSubsystem();
        }
        return instance;
    }

    private CANSparkFlex feederMotor = new CANSparkFlex(SterilizerConstants.NEO_MOTOR_ID, MotorType.kBrushless);
    private LaserCan laser = new LaserCan(SterilizerConstants.LASER_ID);

    /** Creates a new SterilizerSubsystem. LaserCAN is configured in the GrappleHook app */
    public SterilizerSubsystem() {
        super("SterilizerSubsystem");
    }
    
    /**
     * Returns whether or not there is a note in the subsystem (laser broken)
     * 
     * @return contains a note, empty optional if invalid measurements
     */
    public Optional<Boolean> hasNote() {
        LaserCan.Measurement measurement = laser.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return Optional.ofNullable(measurement.distance_mm <= SterilizerConstants.NOTE_DISTANCE_LASER);
        }
        return Optional.empty();
    }
    
    /**
     * Moves the note forwards to the shooter
     */
    public void moveForward() {
        feederMotor.set(SterilizerConstants.FEEDING_SPEED);
    }
    
    /**
     * Moves the note backwards to the intake. Do not run for too long, otherwise the note will get stuck above the PDH.
     */
    public void moveBackward() {
        feederMotor.set(-SterilizerConstants.FEEDING_SPEED);
    }
    
    /**
     * Stops the NEO motor
     */
    public void moveStop() {
        feederMotor.set(0);
    }

    @Override
    public void periodic() {
        LEDSubsystem.getInstance().setDefaultLightState(
            this.hasNote().isPresent() && this.hasNote().get() ? LightState.HOLDING_NOTE : LightState.OFF);
    }
}
