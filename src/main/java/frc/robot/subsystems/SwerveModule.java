package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.SwerveKinematics;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {
    // Instances of the drivng and turning motor for the module
    private CANSparkMax driveMotor;
    private CANSparkMax turningMotor;

    // Instance of the encoder used for turning of the module
    private CANcoder turningEncoder;

    // Instance of PIDcontroller - used to calculate value for turning motor
    private PIDController turningPidController;

    // Instances of values used to help calculate encoder positions for the turning motor
    private boolean absoluteEncoderReversed;

    /**
    * Creates an Instance of the Swerve Module with the specified options
    * 
    * @param driveMotorID            - CAN ID for the driving SparkMax
    * @param turningMotorID          - CAN ID for the turning SparkMax
    * @param turningEncoderID        - CAN ID for the turning encoder (on the
    *                                swerve CAN bus)
    * @param driveMotorReversed      - is the driving motor inverted?
    * @param turningMotorReversed    - is the turning motor inverted?
    * @param absoluteEncoderReversed - is the turning encoder inverted?
    */
    public SwerveModule(int driveMotorID, int turningMotorID, int turningEncoderID,
            boolean driveMotorReversed, boolean turningMotorReversed,
            boolean absoluteEncoderReversed) {
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        // Initializes the driving and turning motor
        this.driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        this.driveMotor.setIdleMode(IdleMode.kBrake);
        this.driveMotor.setInverted(driveMotorReversed);

        this.turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
        this.turningMotor.setIdleMode(IdleMode.kBrake);
        this.turningMotor.setInverted(turningMotorReversed);

        // Initializes the turning encoder on the specific CAN bus
        this.turningEncoder = new CANcoder(turningEncoderID, SwerveModuleConstants.SWERVE_CAN_BUS);

        // Initializes the PID controller using the determined values
        this.turningPidController = new PIDController(
            SwerveKinematics.TURNING_PID_CONTROLLER.KP,
            SwerveKinematics.TURNING_PID_CONTROLLER.KI,
            SwerveKinematics.TURNING_PID_CONTROLLER.KD);

        // Makes the values continuous, so that 0 == 360 degrees
        this.turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
    * Returns the turning position of the CANcoder
    * 
    * @return position of the turning CANcoder (in radians)
    */
    public double getTurningPosition() {
        return getAbsoluteEncoderRad();
    }

    /**
    * Returns the speed of the turning encoder
    * 
    * @return speed of the turning encoder (radians/second)
    */
    public double getTurningVelocity() {
        // Gets velocity as rotations/second
        double velocity = this.turningEncoder.getVelocity().getValueAsDouble();
        // Turn rotations into radians
        return Units.rotationsToRadians(velocity);
    }

    /**
    * Returns the turning position of the CANcoder
    * 
    * @return position of the turning CANcoder (in radians)
    */
    public double getAbsoluteEncoderRad() {
        // Gets position as rotation
        double angle = this.turningEncoder.getAbsolutePosition().getValueAsDouble();
        // Turn rotations to radians
        return Units.rotationsToRadians(angle) * (this.absoluteEncoderReversed ? -1.0 : 1.0);
    }

    /**
    * Returns the current state of the swerve module
    * 
    * @return current state of the swerve module
    */
    public SwerveModuleState getState() {
        // Gets drive velocity as rotations/min (CANSparkMax)
        double velocity = this.driveMotor.getEncoder().getVelocity();
        // The ratio turns rotations/min into radians/second
        // it also makes sure that the odometry getting values from this gets meters correctly
        velocity *= PhysicalConstants.SWERVE_WHEEL_DIAMETER * PhysicalConstants.SWERVE_MOTOR_TO_WHEEL_RATIO;
        return new SwerveModuleState(velocity, new Rotation2d(getTurningPosition()));
    }
  
    /**
    * Returns the current position of the swerve module
    *
    * @return current state of the swerve module
    */
    public SwerveModulePosition getPosition() {
        // Gets drive position as rotations
        double positionRot = this.driveMotor.getEncoder().getPosition();
        // Turns rotations to radians
        double positionMeters = positionRot * PhysicalConstants.SWERVE_WHEEL_DIAMETER * PhysicalConstants.SWERVE_MOTOR_TO_WHEEL_RATIO;
        return new SwerveModulePosition(positionMeters, new Rotation2d(getTurningPosition()));
    }

    /**
     * Zeros the position of the driving encoder
     */
    public void zeroDriveEncoder() {
        this.driveMotor.getEncoder().setPosition(0);
    }

    /**
    * Sets the current state to a desired state
    * 
    * @param state - desired state of the swerve module
    */
    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);

        double driveMotorSpeed = state.speedMetersPerSecond / SwerveKinematics.PHYSICAL_MAX_MODULE_SPEED;
        double turnMotorSpeed = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());

        driveMotor.set(driveMotorSpeed);
        turningMotor.set(turnMotorSpeed);
    }

    /**
    * Stops the swerve module by stopping both the turning and driving motor
    */
    public void stop() {
        this.driveMotor.set(0);
        this.turningMotor.set(0);
    }
      
    /**
    * Gets the voltage of the driving motor
    *
    * @return drive voltage
    */
    public double getDriveVoltage() {
        return this.driveMotor.getBusVoltage() * this.driveMotor.getAppliedOutput();
    }
      
    /** Gets the voltage of the turning motor
    *
    * @return turn voltage
    */
    public double getTurnVoltage() {
        return this.turningMotor.getBusVoltage() * this.turningMotor.getAppliedOutput();
    }
}
