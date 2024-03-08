// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class ShooterSubsystem extends SubsystemBase {    
    // Singleton Design Pattern
    private static ShooterSubsystem instance;
    public static ShooterSubsystem getInstance() {
        if(instance == null) {
            instance = new ShooterSubsystem();
        }
        return instance;
    }

    /** This is used after a pivot command to be sure the robot can chain a shooting command */
    public boolean canShoot = false;

    // private CANSparkFlex rightShooter = new CANSparkFlex(ShooterConstants.RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    // private CANSparkFlex leftShooter = new CANSparkFlex(ShooterConstants.LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private TalonFX rightPivotMotor = new TalonFX(ShooterConstants.LEFT_PIVOT_MOTOR_ID, SwerveModuleConstants.SWERVE_CAN_BUS);
    private TalonFX leftPivotMotor = new TalonFX(ShooterConstants.RIGHT_PIVOT_MOTOR_ID, SwerveModuleConstants.SWERVE_CAN_BUS);
    
    /** Creates a new ShooterSubsystem and configures Motion Magic for the pivot */
    public ShooterSubsystem() {
        // leftShooter.setInverted(true);
        
        configureMotionMagic();
        // Reset absolute position (ONLY DO THIS WITH THE PIVOT VERTICAL)
        // leftPivotMotor.setPosition(0.25 * ShooterConstants.MOTOR_TO_PIVOT_RATIO);
        // rightPivotMotor.setPosition(0.25 * ShooterConstants.MOTOR_TO_PIVOT_RATIO);
    }

    /**
     * Configures motion magic for the intake pivot talon
     */
    private void configureMotionMagic() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        
        FeedbackConfigs feedbackConfigs = configuration.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // Sets the gear ratio from the motor to the mechanism (pivot)
        feedbackConfigs.SensorToMechanismRatio = 0;
        
        MotorOutputConfigs motorOutputConfigs = configuration.MotorOutput;
        motorOutputConfigs.DutyCycleNeutralDeadband = 0.001;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Right motor not inverted
        
        // Not sure what these do, but we shouldn't need to change the update frequencies. If needed, the way to do it is detailed here
        this.rightPivotMotor.getPosition().setUpdateFrequency(50); // 50 hz or 20 ms update time, same as robot loop
        this.leftPivotMotor.getPosition().setUpdateFrequency(50); // 50 hz or 20 ms update time, same as robot loop
        
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
        
        this.rightPivotMotor.getConfigurator().apply(configuration);
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Left motor inverted
        this.leftPivotMotor.getConfigurator().apply(configuration);
    }
    
    /**
     * Goes to the position of the pivot using Motion Magic slot 0
     * 
     * @param position in degrees
     */
    public void pivotGoToPosition(double position) {
        position = MathUtil.clamp(position, ShooterConstants.PIVOT_ANGLE_LIMITS[0], ShooterConstants.PIVOT_ANGLE_LIMITS[1]);
        MotionMagicVoltage control = motionMagicVoltage
        // Select Slot 0 for Motion Magic (should be done by default)
        .withSlot(0)
        .withPosition(Units.degreesToRotations(position * ShooterConstants.MOTOR_TO_PIVOT_RATIO));
        rightPivotMotor.setControl(control);
        leftPivotMotor.setControl(control);
    }

    /**
     * Set the pivot speeds (last resort) between -1.0 and 1.0
     */
    public void setPivotSpeed(double speed) {
        // if (speed > 0 && getPivotPosition() >= ShooterConstants.PIVOT_ANGLE_LIMITS[1]) return;
        // if (speed < 0 && getPivotPosition() <= ShooterConstants.PIVOT_ANGLE_LIMITS[0]) return;
        if(speed > 0 && getPivotPosition() < ShooterConstants.PIVOT_ANGLE_LIMITS[1] || 
        speed < 0 && getPivotPosition() > ShooterConstants.PIVOT_ANGLE_LIMITS[0]) {
          rightPivotMotor.set(speed);
          leftPivotMotor.set(speed);
        }
    }

    /**
     * Testing method to be removed later
     */
    public void TESTING_RESET_PIVOT_POSITION() {
        rightPivotMotor.setPosition(0);
        leftPivotMotor.setPosition(0);
    }

    /**
     * Gets the position of the pivot (after gear ratio) using the motor's encoder (not absolute value)
     * 
     * @return position in degrees
     */
    public double getPivotPosition() {
        return Units.rotationsToDegrees(rightPivotMotor.getPosition().getValueAsDouble() / ShooterConstants.MOTOR_TO_PIVOT_RATIO);
    }

    /**
     * Gets the velocities of the shooter motors. Left is [0] and right is [1]
     * 
     * @return velocities in rad/s
     */
    public double[] getShootingVelocities() {
        // return new double[]{
        //     Units.rotationsPerMinuteToRadiansPerSecond(leftShooter.getEncoder().getVelocity()),
        //     Units.rotationsPerMinuteToRadiansPerSecond(rightShooter.getEncoder().getVelocity())
        // };
        return new double[2];
    }

    /**
     * Sets the velocities of the shooter motors. Left is [0] and right is [1]
     * 
     * @param velocities between -1.0 and 1.0
     */
    public void setShootingVelocities(double[] velocities) {
        // leftShooter.set(velocities[0]);
        // rightShooter.set(velocities[1]);
    }

    /**
     * Stops the shooting motors (overloaded)
     */
    public void setShootingVelocities() {
        setShootingVelocities(new double[2]);
    }

    @Override
    public void periodic() {}
}
