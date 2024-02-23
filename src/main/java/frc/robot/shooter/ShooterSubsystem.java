// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    // Singleton Design Pattern
    private static SterilizerSubsystem instance;
    public static SterilizerSubsystem getInstance() {
        if(instance == null) {
            instance = new SterilizerSubsystem();
        }
        return instance;
    }

    private CANSparkFlex rightShooter = new CANSparkFlex(ShooterConstants.RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    private CANSparkFlex leftShooter = new CANSparkFlex(ShooterConstants.LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private TalonFX leaderPivot = new TalonFX(ShooterConstants.RIGHT_PIVOT_MOTOR_ID);
    private TalonFX followerPivot = new TalonFX(ShooterConstants.LEFT_PIVOT_MOTOR_ID);

    /** Creates a new ShooterSubsystem.*/
    public ShooterSubsystem() {
        leftShooter.setInverted(true);
        // https://v6.docs.ctr-electronics.com/en/2023-v6/docs/migration/migration-guide/control-requests-guide.html#follower-motors
        followerPivot.setControl(new Follower(leaderPivot.getDeviceID(), true));
        
        configureMotionMagic();
    }

    /**
     * Configures motion magic for the intake pivot talon
     */
    private void configureMotionMagic() {
        TalonFXConfigurator configurator = this.leaderPivot.getConfigurator();
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        
        FeedbackConfigs feedbackConfigs = configuration.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // Sets the gear ratio from the motor to the mechanism (pivot)
        feedbackConfigs.SensorToMechanismRatio = ShooterConstants.MOTOR_TO_PIVOT_RATIO;
        
        MotorOutputConfigs motorOutputConfigs = configuration.MotorOutput;
        motorOutputConfigs.DutyCycleNeutralDeadband = 0.001;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Not inverted
        
        // Not sure what these do, but we shouldn't need to change the update frequencies. If needed, the way to do it is detailed here
        this.leaderPivot.getPosition().setUpdateFrequency(50); // 50 hz or 20 ms update time, same as robot loop
        // https://v6.docs.ctr-electronics.com/en/2023-v6/docs/api-reference/api-usage/status-signals.html#changing-update-frequency
        // https://v6.docs.ctr-electronics.com/en/2023-v6/docs/migration/migration-guide/status-signals-guide.html#changing-update-frequency-status-frame-period
        // this.leaderPivot.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
        // this.leaderPivot.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);
        
        // Set Motion Magic gains in slot0
        Slot0Configs slot0Configs = configuration.Slot0;
        slot0Configs.kS = ShooterConstants.SLOT_0_CONFIGS.kS;
        slot0Configs.kV = ShooterConstants.SLOT_0_CONFIGS.kV;
        slot0Configs.kP = ShooterConstants.SLOT_0_CONFIGS.kP;
        slot0Configs.kI = ShooterConstants.SLOT_0_CONFIGS.kI;
        slot0Configs.kD = ShooterConstants.SLOT_0_CONFIGS.kD;
        
        // Set acceleration and vcruise velocity
        MotionMagicConfigs motionMagicConfigs = configuration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ShooterConstants.CRUISE_SPEED;
        motionMagicConfigs.MotionMagicAcceleration = ShooterConstants.CRUISE_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = ShooterConstants.MOTION_MAGIC_JERK;
        
        // Shouldn't need to zero the sensor due to using Tuner X configured absolute 0 
        // Talon is always initialized to absolute position
        // this.leaderPivot.setSelectedSensorPosition(0, ShooterConstants.PID_LOOP_IDX, ShooterConstants.TIMEOUT_MS);

        configurator.apply(configuration);
    }

    /**
     * Sets the position of the pivot using Motion Magic slot 0
     * 
     * @param position in degrees
     */
    public void setPivotPosition(double position) {
        // Select Slot 0 for Motion Magic
        motionMagicVoltage.Slot = 0;
        leaderPivot.setControl(motionMagicVoltage.withPosition(Units.degreesToRotations(position)));
    }
    
    /**
     * Gets the velocities of the shooter motors. Left is [0] and right is [1]
     * 
     * @return velocities in rad/s
     */
    public double[] getShootingVelocities() {
        return new double[]{
            Units.rotationsPerMinuteToRadiansPerSecond(leftShooter.getEncoder().getVelocity()),
            Units.rotationsPerMinuteToRadiansPerSecond(rightShooter.getEncoder().getVelocity())
        };
    }

    /**
     * Sets the velocities of the shooter motors. Left is [0] and right is [1]
     * 
     * @param velocities between -1.0 and 1.0
     */
    public void setShootingVelocities(double[] velocities) {
        leftShooter.set(velocities[0]);
        rightShooter.set(velocities[1]);
    }

    @Override
    public void periodic() {}
}
