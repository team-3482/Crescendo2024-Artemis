package frc.robot.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.PhysicalConstants.RobotConstants;
import frc.robot.constants.PhysicalConstants.SwerveKinematics;
import frc.robot.constants.PrimeNumbers;

/**
 * A class to represent each swerve module and its components.
 */
public class SwerveModule {
    private CANSparkMax driveMotor;
    private CANSparkMax turningMotor;

    private CANcoder turningEncoder;

    /** Calculations are in radians. */
    private PIDController turningPidController;

    private boolean absoluteEncoderReversed;

    /**
     * Creates a SwerveModule.
     * @param driveMotorID            - CAN ID for the driving SparkMax.
     * @param turningMotorID          - CAN ID for the turning SparkMax.
     * @param turningEncoderID        - CAN ID for the turning encoder (on the swerve CAN bus).
     * @param driveMotorReversed      - is the driving motor inverted?
     * @param turningMotorReversed    - is the turning motor inverted?
     * @param absoluteEncoderReversed - is the turning encoder inverted?
     */
    public SwerveModule(
        int driveMotorID, int turningMotorID, int turningEncoderID,
        boolean driveMotorReversed, boolean turningMotorReversed,
        boolean absoluteEncoderReversed
    ) {
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        // Initializes the driving and turning motor.
        this.driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        this.driveMotor.setIdleMode(IdleMode.kBrake);
        this.driveMotor.setInverted(driveMotorReversed);

        this.turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
        this.turningMotor.setIdleMode(IdleMode.kBrake);
        this.turningMotor.setInverted(turningMotorReversed);

        // Initializes the turning encoder on the specific CAN bus.
        this.turningEncoder = new CANcoder(turningEncoderID, RobotConstants.SWERVE_CAN_BUS);

        // Initializes the PID controller using the determined values.
        this.turningPidController = new PIDController(
            SwerveKinematics.TURNING_PID_CONTROLLER.KP,
            SwerveKinematics.TURNING_PID_CONTROLLER.KI,
            SwerveKinematics.TURNING_PID_CONTROLLER.KD
        );

        // Makes the values continuous, because input from the CANcoders
        // is from -180 to 180, and they are equal.
        this.turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        setStatusFrames();
    }

    /**
     * Gets position of the turning CANcoder.
     * @return position in radians.
     */
    public double getTurningPosition() {
        // Gets position as a rotation.
        double angle = this.turningEncoder.getAbsolutePosition().getValueAsDouble();
        // Turn rotations to radians.
        return Units.rotationsToRadians(angle) * (this.absoluteEncoderReversed ? -1.0 : 1.0);
    }

    /**
     * Gets the speed of the turning CANcoder.
     * @return speed in radians/second.
     */
    public double getTurningVelocity() {
        // Gets velocity as rotations/second.
        double velocity = this.turningEncoder.getVelocity().getValueAsDouble();
        // Turn rotations into radians.
        return Units.rotationsToRadians(velocity);
    }

    /**
     * Gets the current state of the swerve module.
     * @return state.
     */
    public SwerveModuleState getState() {
        // Gets drive velocity as rotations/min (CANSparkMax).
        double velocity = this.driveMotor.getEncoder().getVelocity();
        // The ratio turns rotations/min into radians/second.
        // It also makes sure that the odometry getting values from this gets meters correctly.
        velocity *= RobotConstants.SWERVE_WHEEL_DIAMETER * RobotConstants.SWERVE_MOTOR_TO_WHEEL_RATIO;
        return new SwerveModuleState(velocity, new Rotation2d(getTurningPosition()));
    }
  
    /**
     * Gets the current position of the swerve module.
     * @return position.
     */
    public SwerveModulePosition getPosition() {
        // Gets drive position as rotations.
        double positionRot = this.driveMotor.getEncoder().getPosition();
        // Turns rotations to radians.
        double positionMeters = positionRot * RobotConstants.SWERVE_WHEEL_DIAMETER * RobotConstants.SWERVE_MOTOR_TO_WHEEL_RATIO;
        return new SwerveModulePosition(positionMeters, new Rotation2d(getTurningPosition()));
    }

    /**
     * Zeros the position of the driving encoder.
     */
    public void zeroDriveEncoder() {
        this.driveMotor.getEncoder().setPosition(0);
    }

    /**
     * Sets the current state to a desired state.
     * @param state desired state.
     */
    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);

        double driveMotorSpeed = state.speedMetersPerSecond / SwerveKinematics.PHYSICAL_MAX_MODULE_SPEED;
        double turnMotorSpeed = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());

        driveMotor.set(driveMotorSpeed);
        turningMotor.set(turnMotorSpeed);
    }

    /**
     * Stops the swerve module.
     */
    public void stop() {
        this.driveMotor.set(0);
        this.turningMotor.set(0);
    }

    /**
     * Limits the publishing of CAN messages to the bus.
     */
    private void setStatusFrames() {
        // driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        // driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 20);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, PrimeNumbers.getNextPrimeNumber());
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, PrimeNumbers.getNextPrimeNumber());
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, PrimeNumbers.getNextPrimeNumber());
        // driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 250);
        
        // turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, PrimeNumbers.getNextPrimeNumber());
        turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, PrimeNumbers.getNextPrimeNumber());
        turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, PrimeNumbers.getNextPrimeNumber());
        turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, PrimeNumbers.getNextPrimeNumber());
        turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, PrimeNumbers.getNextPrimeNumber());
        turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, PrimeNumbers.getNextPrimeNumber());
        // turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 250);
        
    }
}
