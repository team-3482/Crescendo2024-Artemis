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
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.RobotConstants;
import frc.robot.constants.PhysicalConstants.ShooterConstants;
import frc.robot.constants.PrimeNumbers;

/**
 * A subsystem that moves and controls the shooter.
 */
public class ShooterSubsystem extends SubsystemBase {    
    // Thread-safe singleton design pattern.
    private static volatile ShooterSubsystem instance;
    private static Object mutex = new Object();
    public static ShooterSubsystem getInstance() {
        ShooterSubsystem result = instance;
        if(result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new ShooterSubsystem();
                }
            }
        }
        return instance;
    }

    /**
     * This subsystem is used only for Command requirements.
     */
    private class PivotRequirement extends SubsystemBase {
        public PivotRequirement() {
            setName("Shooter - Pivot Requirement");
        }
    }

    /**
     * This subsystem is used only for Command requirements.
     */
    private class ShootingRequirement extends SubsystemBase {
        public ShootingRequirement() {
            setName("Shooter - Shooting Requirement");
        }
    }

    private PivotRequirement pivotRequirement = new PivotRequirement();
    private ShootingRequirement shootingRequirement = new ShootingRequirement();

    // Shooting wheels
    private CANSparkFlex rightShooter = new CANSparkFlex(ShooterConstants.RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    private SparkPIDController rightPID = rightShooter.getPIDController();
    private CANSparkFlex leftShooter = new CANSparkFlex(ShooterConstants.LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    private SparkPIDController leftPID = leftShooter.getPIDController();
    
    // Pivot
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private TalonFX rightPivotMotor = new TalonFX(ShooterConstants.LEFT_PIVOT_MOTOR_ID, RobotConstants.SWERVE_CAN_BUS);
    private TalonFX leftPivotMotor = new TalonFX(ShooterConstants.RIGHT_PIVOT_MOTOR_ID, RobotConstants.SWERVE_CAN_BUS);
    private CANcoder rightCANcoder = new CANcoder(ShooterConstants.RIGHT_CANCODER_ID, RobotConstants.SWERVE_CAN_BUS);
    private CANcoder leftCANcoder = new CANcoder(ShooterConstants.LEFT_CANCODER_ID, RobotConstants.SWERVE_CAN_BUS);

    /**
     * Creates a new ShooterSubsystem, and configures motors.
     */
    public ShooterSubsystem() {
        super("ShooterSubsystem");
        
        // leftShooter.setInverted(true);
        
        configureMotionMagic();
        configureShootingPID();
        setRotorPositions();

        setStatusFrames();
    }

    @Override
    public void periodic() {}

    /**
     * Configures motion magic for the shooter pivot Talons.
     */
    private void configureMotionMagic() {
        // Shared configurations
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        
        FeedbackConfigs feedbackConfigs = configuration.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        // Sets the gear ratio from the motor to the mechanism (pivot).
        // This is 1 because we use MotionMagic with the rotor,
        // and we multiply all inputs to go to the right rotor position.
        feedbackConfigs.SensorToMechanismRatio = 1;
        
        MotorOutputConfigs motorOutputConfigs = configuration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        
        // Set Motion Magic gains in slot 0.
        Slot0Configs slot0Configs = configuration.Slot0;
        slot0Configs.kS = ShooterConstants.slot0Configs.kS;
        slot0Configs.kV = ShooterConstants.slot0Configs.kV;
        slot0Configs.kP = ShooterConstants.slot0Configs.kP;
        slot0Configs.kI = ShooterConstants.slot0Configs.kI;
        slot0Configs.kD = ShooterConstants.slot0Configs.kD;
        
        // Set acceleration and cruise velocity.
        MotionMagicConfigs motionMagicConfigs = configuration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ShooterConstants.CRUISE_SPEED;
        motionMagicConfigs.MotionMagicAcceleration = ShooterConstants.CRUISE_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = ShooterConstants.MOTION_MAGIC_JERK;
        
        // Motor-specific configurations.
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Right motor not inverted.
        feedbackConfigs.FeedbackRemoteSensorID = this.rightCANcoder.getDeviceID();
        this.rightPivotMotor.getConfigurator().apply(configuration);
        
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Left motor inverted.
        feedbackConfigs.FeedbackRemoteSensorID = this.leftCANcoder.getDeviceID();
        this.leftPivotMotor.getConfigurator().apply(configuration);
    }

    /**
     * Configures PID for both shooting motors.
     */
    private void configureShootingPID() {
        // Left shooter
        leftPID.setP(ShooterConstants.Shooting.kP_SHOOTING);
        leftPID.setFF(ShooterConstants.Shooting.kFF_SHOOTING);
        leftPID.setOutputRange(-1, 1);
        
        // Right shooter
        rightPID.setP(ShooterConstants.Shooting.kP_SHOOTING);
        rightPID.setFF(ShooterConstants.Shooting.kFF_SHOOTING);
        rightPID.setOutputRange(-1, 1);
    }
    
    /**
     * Goes to provided pivot position using Motion Magic slot 0.
     * @param position for the pivot in degrees.
     * @see {@link ShooterSubsystem#setRotorPositions()} to reset rotor positions for best accuracy.
     * @apiNote The position is clamped by {@link ShooterConstants#ANGLE_LIMITS}.
     */
    public void pivotGoToPosition(double position) {
        position = MathUtil.clamp(position, ShooterConstants.Pivot.ANGLE_LIMITS[0], ShooterConstants.Pivot.ANGLE_LIMITS[1]);

        MotionMagicVoltage control = motionMagicVoltage
            // Select Slot 0 for Motion Magic (should be done by default).
            .withSlot(0)
            .withPosition(Units.degreesToRotations(position * ShooterConstants.Pivot.MOTOR_TO_PIVOT_RATIO));

        rightPivotMotor.setControl(control);
        leftPivotMotor.setControl(control);
    }

    /**
     * Set the pivot speeds for each motor (last resort) between -1.0 and 1.0.
     * @param leftSpeed speed for the left motor. Positive is up.
     * @param rightSpeed speed for the right motor. Positive is up.
     * @param override the soft limits.
     * @apiNote Will respect soft limits for each pivot at {@link ShooterConstants#PIVOT_ANGLE_LIMITS}.
     */
    public void setPivotSpeed(double leftSpeed, double rightSpeed, boolean override) {
        if (!override) {
            double[] positions = getCANcoderPositions();
            leftSpeed = (leftSpeed < 0 && positions[0] <= ShooterConstants.Pivot.ANGLE_LIMITS[0]) ||
                (leftSpeed > 0 && positions[0] >= ShooterConstants.Pivot.ANGLE_LIMITS[1]) ? 0 : leftSpeed;
            rightSpeed = (rightSpeed < 0 && positions[1] <= ShooterConstants.Pivot.ANGLE_LIMITS[0]) ||
                (rightSpeed > 0 && positions[1] >= ShooterConstants.Pivot.ANGLE_LIMITS[1]) ? 0 : rightSpeed;
        }
        
        rightPivotMotor.set(rightSpeed);
        leftPivotMotor.set(leftSpeed);
    }

    /**
     * Set the pivot speed for both motors (last resort) between -1.0 and 1.0.
     * @param speed for both motors. Positive is up.
     * @param override the soft limits.
     * @apiNote Will respect soft limits for each pivot at {@link ShooterConstants#PIVOT_ANGLE_LIMITS}.
     */
    public void setPivotSpeed(double speed, boolean override) {
        setPivotSpeed(speed, speed, override);
    }

    /**
     * Stops the shooter pivot.
     */
    public void setPivotSpeed() {
        setPivotSpeed(0, true);
    }
    
    /**
     * Gets the positions of the pivots using the CANCoders.
     * @return positions in degrees, left [0] and right [1].
     */
    public double[] getCANcoderPositions() {
        return new double[]{
            Units.rotationsToDegrees(leftCANcoder.getPosition().getValueAsDouble()),
            Units.rotationsToDegrees(rightCANcoder.getPosition().getValueAsDouble())
        };
    }
    
    /**
     * Sets the positions of the rotors.
     * @param positions in degrees, left [0] and right [1].
     * @apiNote Will multiply input values by the gear ratio.
     */
    public void setRotorPositions(double[] positions) {
        leftPivotMotor.setPosition(Units.degreesToRotations(positions[0] * ShooterConstants.Pivot.MOTOR_TO_PIVOT_RATIO));
        rightPivotMotor.setPosition(Units.degreesToRotations(positions[1] * ShooterConstants.Pivot.MOTOR_TO_PIVOT_RATIO));
    }
    
    /**
     * Sets the positions of the rotors using the CANcoders.
     */
    public void setRotorPositions() {
        setRotorPositions(getCANcoderPositions());
    }

    /**
     * Gets the velocities of the shooting motors.
     * @return velocities in RPM, left [0] and right [1].
     */
    public double[] getShootingVelocities() {
        return new double[]{
            leftShooter.getEncoder().getVelocity(),
            rightShooter.getEncoder().getVelocity()
        };
    }

    /**
     * Sets the velocities of the shooting motors. 
     * @param velocities between -1.0 and 1.0, left [0] and right [1].
     */
    public void setShootingVelocities(double[] velocities) {
        leftPID.setReference(velocities[0], ControlType.kVelocity);
        rightPID.setReference(velocities[1],  ControlType.kVelocity);
    }

    /**
     * Stops the shooting motors.
     */
    public void setShootingVelocities() {
        setShootingVelocities(new double[]{0, 0});
    }

    /**
     * Returns a subsystem to be used with Command requirements.
     * @return pivot subsystem.
     */
    public Subsystem getPivotRequirement() {
        return this.pivotRequirement;
    }

    /**
     * Returns a subsystem to be used with Command requirements.
     * @return shooting subsystem.
     */
    public Subsystem getShootingRequirement() {
        return this.shootingRequirement;
    }

    /**
     * Limits the publishing of CAN messages to the bus.
     */
    private void setStatusFrames() {
        // leftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        // leftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        leftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, PrimeNumbers.getNextPrimeNumber());
        // leftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
        leftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus4, PrimeNumbers.getNextPrimeNumber());
        leftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus5, PrimeNumbers.getNextPrimeNumber());
        // leftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200);
        // leftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 250);
        
        // rightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        // rightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        rightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, PrimeNumbers.getNextPrimeNumber());
        // rightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
        rightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus4, PrimeNumbers.getNextPrimeNumber());
        rightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus5, PrimeNumbers.getNextPrimeNumber());
        // rightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200);
        // rightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 250);
    }
}
