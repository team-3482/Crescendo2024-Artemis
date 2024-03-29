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
import frc.robot.constants.PhysicalConstants.RobotConstants;
import frc.robot.constants.PhysicalConstants.ShooterConstants;

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
    private TalonFX rightPivotMotor = new TalonFX(ShooterConstants.LEFT_PIVOT_MOTOR_ID, RobotConstants.SWERVE_CAN_BUS);
    private TalonFX leftPivotMotor = new TalonFX(ShooterConstants.RIGHT_PIVOT_MOTOR_ID, RobotConstants.SWERVE_CAN_BUS);
    private CANcoder rightCANcoder = new CANcoder(ShooterConstants.RIGHT_CANCODER_ID, RobotConstants.SWERVE_CAN_BUS);
    private CANcoder leftCANcoder = new CANcoder(ShooterConstants.LEFT_CANCODER_ID, RobotConstants.SWERVE_CAN_BUS);

    /** Creates a new ShooterSubsystem, sets pivot positions, and configures Motion Magic for the pivot */
    public ShooterSubsystem() {
        super("ShooterSubsystem");
        // leftShooter.setInverted(true);
        
        configureMotionMagic();
        configureShootingPID();
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
        feedbackConfigs.SensorToMechanismRatio = 1;
        
        MotorOutputConfigs motorOutputConfigs = configuration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        
        // Set Motion Magic gains in slot0
        Slot0Configs slot0Configs = configuration.Slot0;
        slot0Configs.kS = ShooterConstants.slot0Configs.kS;
        slot0Configs.kV = ShooterConstants.slot0Configs.kV;
        slot0Configs.kP = ShooterConstants.slot0Configs.kP;
        slot0Configs.kI = ShooterConstants.slot0Configs.kI;
        slot0Configs.kD = ShooterConstants.slot0Configs.kD;
        
        // Set acceleration and vcruise velocity
        MotionMagicConfigs motionMagicConfigs = configuration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ShooterConstants.CRUISE_SPEED;
        motionMagicConfigs.MotionMagicAcceleration = ShooterConstants.CRUISE_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = ShooterConstants.MOTION_MAGIC_JERK;
        
        // Motor-specific configurations
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Right motor not inverted
        feedbackConfigs.FeedbackRemoteSensorID = this.rightCANcoder.getDeviceID();
        this.rightPivotMotor.getConfigurator().apply(configuration);
        
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Left motor inverted
        feedbackConfigs.FeedbackRemoteSensorID = this.leftCANcoder.getDeviceID();
        this.leftPivotMotor.getConfigurator().apply(configuration);
    }

    /**
     * Configures PID for both shooting motors
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
     * Goes to the position of the pivot using Motion Magic slot 0
     * @deprecated MotionMagic is broken with CANCoder's innacurate velocities (vibration).
     * Use {@link ShooterSubsystem#setPivotSpeed(double, boolean)} instead.
     * @param position in degrees
     * @apiNote The position is clamped by {@link ShooterConstants#ANGLE_LIMITS}
     */
    public void pivotGoToPosition(double position) {
        position = MathUtil.clamp(position, ShooterConstants.Pivot.ANGLE_LIMITS[0], ShooterConstants.Pivot.ANGLE_LIMITS[1]);

        MotionMagicVoltage control = motionMagicVoltage
            // Select Slot 0 for Motion Magic (should be done by default)
            .withSlot(0)
            .withPosition(Units.degreesToRotations(position));

        rightPivotMotor.setControl(control);
        leftPivotMotor.setControl(control);
    }

    /**
     * Set the pivot speeds (last resort) between -1.0 and 1.0.
     * Will set the speed to 0 for each motor individually per {@link ShooterConstants} {@code PIVOT_ANGLE_LIMITS}
     * @param leftSpeed speed for the left motor
     * @param rightSpeed speed for the right motor
     * @param override the soft limits
     */
    public void setPivotSpeed(double leftSpeed, double rightSpeed, boolean override) {
        if (!override) {
            double[] positions = getPivotPositions();
            leftSpeed = (leftSpeed < 0 && positions[0] <= ShooterConstants.Pivot.ANGLE_LIMITS[0]) ||
                (leftSpeed > 0 && positions[0] >= ShooterConstants.Pivot.ANGLE_LIMITS[1]) ? 0 : leftSpeed;
            rightSpeed = (rightSpeed < 0 && positions[1] <= ShooterConstants.Pivot.ANGLE_LIMITS[0]) ||
                (rightSpeed > 0 && positions[1] >= ShooterConstants.Pivot.ANGLE_LIMITS[1]) ? 0 : rightSpeed;
            }
        
        rightPivotMotor.set(rightSpeed);
        leftPivotMotor.set(leftSpeed);
    }

    /**
     * Set the pivot speeds (last resort) between -1.0 and 1.0.
     * Will set the speed to 0 for each motor individually per {@link ShooterConstants} {@code PIVOT_ANGLE_LIMITS}
     * @param speed for both motors
     * @param override the soft limits
     */
    public void setPivotSpeed(double speed, boolean override) {
        setPivotSpeed(speed, speed, override);
    }
    
    /**
     * Gets the positions of the pivots using the CANCoders.
     * <p> Left [0] and right [1] </p>
     * @return positions in degrees
     */
    public double[] getPivotPositions() {
        return new double[]{
            Units.rotationsToDegrees(leftCANcoder.getPosition().getValueAsDouble()),
            Units.rotationsToDegrees(rightCANcoder.getPosition().getValueAsDouble())
        };
    }

    /**
     * Gets the velocities of the shooter motors. Left is [0] and right is [1]
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

        // double[] pos = getPivotPositions();
        // System.out.println("Left : " + Telemetry.D_FORMAT.format(leftPivotMotor.getRotorPosition().getValueAsDouble())
        //     + " Right : " + Telemetry.D_FORMAT.format(rightPivotMotor.getRotorPosition().getValueAsDouble()));
    }
}
