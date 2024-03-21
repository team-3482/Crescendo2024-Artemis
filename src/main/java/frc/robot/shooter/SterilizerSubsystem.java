// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

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
    private LaserCan backLaser = new LaserCan(SterilizerConstants.BACK_LASER_ID);
    private LaserCan frontLaser = new LaserCan(SterilizerConstants.FRONT_LASER_ID);

    /** Creates a new SterilizerSubsystem. LaserCAN is configured in the GrappleHook app */
    public SterilizerSubsystem() {
        super("SterilizerSubsystem");
    }
    
    /**
     * Gets the distances of notes from each laser.
     * 
     * @return measurements, back laser [0] and front laser [1]
     * @apiNote null when the measurement is invalid.
     */
    public Integer[] getLaserMeasurements() {
        LaserCan.Measurement backMm = backLaser.getMeasurement();
        LaserCan.Measurement frontMm = frontLaser.getMeasurement();
        
        Integer[] measurements = new Integer[]{
            backMm != null && backMm.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT ?
                backMm.distance_mm : null,
            frontMm != null && frontMm.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT ?
                frontMm.distance_mm : null
        };

        return measurements;
    }

    /**
     * Checks the distances of {@link SterilizerSubsystem#getLaserMeasurements()} against the measurements for a note.
     * 
     * @return has notes, back laser [0] and front laser [1]
     * @apiNote null when the measurement is invalid.
     */
    public Boolean[] getHasNotes() {
        Integer[] measurements = getLaserMeasurements();

        return new Boolean[]{
            measurements[0] == null ? null : measurements[0] <= SterilizerConstants.NOTE_DISTANCE_LASER,
            measurements[1] == null ? null : measurements[1] <= SterilizerConstants.NOTE_DISTANCE_LASER
        };
    }

    /**
     * Checks if either of {@link SterilizerSubsystem#getHasNotes()} is true
     * 
     * @return if either laser sees a note
     * @apiNote null if both measurements are invalid
     */
    public Boolean hasNote() {
        Boolean[] notes = getHasNotes();

        return notes[0] == null && notes[1] == null ? null : notes[0] || notes[1];
    }
    
    /**
     * Spins the sterilizer at the given speed
     * 
     * @param speed from -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        feederMotor.set(speed);
    }

    /**
     * Stops the sterilizer (overloaded)
     */
    public void setSpeed() {
        setSpeed(0);
    }

    @Override
    public void periodic() {
        Boolean _hasNote = hasNote();
        if (_hasNote != null && _hasNote) {
            LEDSubsystem.getInstance().setLightState(LightState.HOLDING_NOTE, false);
        } 
    }
}
