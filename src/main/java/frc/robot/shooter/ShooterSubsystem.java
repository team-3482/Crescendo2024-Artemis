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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.utilities.JSONManager;

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

    // Shooting wheels
    private CANSparkFlex rightShooter = new CANSparkFlex(ShooterConstants.RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    private SparkPIDController rightPID = rightShooter.getPIDController();
    private CANSparkFlex leftShooter = new CANSparkFlex(ShooterConstants.LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    private SparkPIDController leftPID = leftShooter.getPIDController();
    
    // Pivot
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private TalonFX rightPivotMotor = new TalonFX(ShooterConstants.LEFT_PIVOT_MOTOR_ID, SwerveModuleConstants.SWERVE_CAN_BUS);
    private TalonFX leftPivotMotor = new TalonFX(ShooterConstants.RIGHT_PIVOT_MOTOR_ID, SwerveModuleConstants.SWERVE_CAN_BUS);
    private CANcoder rightCANcoder = new CANcoder(ShooterConstants.RIGHT_CANCODER_ID, SwerveModuleConstants.SWERVE_CAN_BUS);
    private CANcoder leftCANcoder = new CANcoder(ShooterConstants.LEFT_CANCODER_ID, SwerveModuleConstants.SWERVE_CAN_BUS);

    /** Creates a new ShooterSubsystem, sets pivot positions, and configures Motion Magic for the pivot */
    public ShooterSubsystem() {
        super("ShooterSubsystem");
        // leftShooter.setInverted(true);
        
        configureMotionMagic();
        configureShootingPID();
        
        double[] positions = JSONManager.getInstance().getShooterPivotPositions();
        leftPivotMotor.setPosition(Units.degreesToRotations(positions[0] * ShooterConstants.MOTOR_TO_PIVOT_RATIO));
        rightPivotMotor.setPosition(Units.degreesToRotations(positions[1] * ShooterConstants.MOTOR_TO_PIVOT_RATIO));
    }

    /**
     * Configures motion magic for the intake pivot talon
     */
    private void configureMotionMagic() {
        // Shared configurations
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        
        FeedbackConfigs feedbackConfigs = configuration.Feedback;
        // feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // Sets the gear ratio from the motor to the mechanism (pivot)
        feedbackConfigs.SensorToMechanismRatio = 0;
        
        MotorOutputConfigs motorOutputConfigs = configuration.MotorOutput;
        // motorOutputConfigs.DutyCycleNeutralDeadband = 0.001;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        
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
        
        // Motor-specific configurations
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Right motor not inverted
        feedbackConfigs.FeedbackRemoteSensorID = ShooterConstants.RIGHT_CANCODER_ID;
        this.rightPivotMotor.getConfigurator().apply(configuration);
        
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Left motor inverted
        feedbackConfigs.FeedbackRemoteSensorID = ShooterConstants.LEFT_CANCODER_ID;
        this.leftPivotMotor.getConfigurator().apply(configuration);
    }

    /**
     * Configures PID for both shooting motors
     */
    private void configureShootingPID() {
        // Left shooter
        leftPID.setP(ShooterConstants.kP);
        leftPID.setFF(ShooterConstants.kFF);
        leftPID.setOutputRange(-1, 1);
        
        // Right shooter
        rightPID.setP(ShooterConstants.kP);
        rightPID.setFF(ShooterConstants.kFF);
        rightPID.setOutputRange(-1, 1);
    }

    /**
     * @deprecated This method should not be used. Set CANCoders zeroes in Phoenix Tuner X instead.
     * 
     * Set the motor encoder's position to the pivot angle
     * 
     * @param angle in degrees
     */
    @Deprecated
    public void resetPivotPosition(double angle) {
        // JSONManager.getInstance().saveShooterPivotPositions(angle);
        // double position = Units.degreesToRotations(angle) * ShooterConstants.MOTOR_TO_PIVOT_RATIO;
        // leftPivotMotor.setPosition(position);
        // rightPivotMotor.setPosition(position);
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
     * Set the pivot speeds (last resort) between -1.0 and 1.0.
     * Will set the speed to 0 for each motor individually per {@link ShooterConstants} {@code PIVOT_ANGLE_LIMITS}
     * 
     * @param speed for both motors
     * @param override the soft limits
     */
    public void setPivotSpeed(double speed, boolean override) {
        setPivotSpeed(speed, speed, override);
    }
    /**
     * Set the pivot speeds (last resort) between -1.0 and 1.0.
     * Will set the speed to 0 for each motor individually per {@link ShooterConstants} {@code PIVOT_ANGLE_LIMITS}
     * 
     * @param leftSpeed speed for the left motor
     * @param rightSpeed speed for the right motor
     * @param override the soft limits
     */
    public void setPivotSpeed(double leftSpeed, double rightSpeed, boolean override) {
        if (!override) {
            double[] positions = getPivotPositions();
            leftSpeed = (leftSpeed < 0 && positions[0] <= ShooterConstants.PIVOT_ANGLE_LIMITS[0]) ||
                (leftSpeed > 0 && positions[0] >= ShooterConstants.PIVOT_ANGLE_LIMITS[1]) ? 0 : leftSpeed;
            rightSpeed = (rightSpeed < 0 && positions[1] <= ShooterConstants.PIVOT_ANGLE_LIMITS[0]) ||
                (rightSpeed > 0 && positions[1] >= ShooterConstants.PIVOT_ANGLE_LIMITS[1]) ? 0 : rightSpeed;
        }

        rightPivotMotor.set(rightSpeed);
        leftPivotMotor.set(leftSpeed);
    }

    /**
     * Gets the positions of the pivots using the CANCoders.
     * <p> Left [0] and right [1] </p>
     * 
     * @return positions in degrees
     */
    public double[] getPivotPositions() {
        return new double[]{
            Units.rotationsToDegrees(leftCANcoder.getPosition().getValueAsDouble()),
            Units.rotationsToDegrees(rightCANcoder.getPosition().getValueAsDouble())
        };
        // return new double[]{
        //     Units.rotationsToDegrees(leftPivotMotor.getPosition().getValueAsDouble() / ShooterConstants.MOTOR_TO_PIVOT_RATIO),
        //     Units.rotationsToDegrees(rightPivotMotor.getPosition().getValueAsDouble() / ShooterConstants.MOTOR_TO_PIVOT_RATIO)
        // };
    }

    /**
     * Gets the velocities of the shooter motors. Left is [0] and right is [1]
     * 
     * @return velocities in RPM
     */
    public double[] getShootingVelocities() {
        return new double[]{
            leftShooter.getEncoder().getVelocity(),
            rightShooter.getEncoder().getVelocity()
        };
    }

    /**
     * Sets the velocities of the shooter motors. Left is [0] and right is [1]
     * 
     * @param velocities between -1.0 and 1.0
     */
    public void setShootingVelocities(double[] velocities) {
        leftPID.setReference(velocities[0], ControlType.kVelocity);
        rightPID.setReference(velocities[1],  ControlType.kVelocity);
    }

    /**
     * Stops the shooting motors (overloaded)
     */
    public void setShootingVelocities() {
        setShootingVelocities(new double[2]);
    }

    @Override
    public void periodic() {
        // DecimalFormat d = new DecimalFormat("#.##");
        // double[] vel = getShootingVelocities();
        // System.out.println(
        //     "left vel : " + d.format(vel[0]) + " right vel : "
        //     + d.format(vel[1])
        // );
        // System.out.println("Left : " + (int) Units.rotationsToDegrees(leftCANcoder.getPosition().getValueAsDouble()) +
        //     " Right : " + (int) Units.rotationsToDegrees(rightCANcoder.getPosition().getValueAsDouble()));
    }
}
