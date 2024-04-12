package frc.robot.swerve;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.TelemetryConstants.LoggingTags;
import frc.robot.constants.PhysicalConstants.LimelightConstants;
import frc.robot.constants.PhysicalConstants.RobotConstants;
import frc.robot.constants.PhysicalConstants.SwerveKinematics;
import frc.robot.constants.PhysicalConstants.SwerveModuleConfigs;
import frc.robot.limelight.LimelightHelpers;
import frc.robot.limelight.LimelightSubsystem;
import frc.robot.utilities.SwerveUtilities;
import frc.robot.utilities.Telemetry;

/**
 * A subsystem used for moving the robot.
 */
public class SwerveSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile SwerveSubsystem instance;
    private static Object mutex = new Object();

    public static SwerveSubsystem getInstance() {
        SwerveSubsystem result = instance;
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null)
                    instance = result = new SwerveSubsystem();
            }
        }
        return instance;
    }

    private SwerveModule moduleOne = new SwerveModule(
        SwerveModuleConfigs.One.DRIVE,
        SwerveModuleConfigs.One.TURN,
        SwerveModuleConfigs.One.ENCODER,
        SwerveModuleConfigs.One.DRIVE_MOTOR_REVERSED,
        SwerveModuleConfigs.One.TURNING_MOTOR_REVERSED,
        SwerveModuleConfigs.One.ABSOLUTE_ENCODER_REVERSED
    );

    private SwerveModule moduleTwo = new SwerveModule(
        SwerveModuleConfigs.Two.DRIVE,
        SwerveModuleConfigs.Two.TURN,
        SwerveModuleConfigs.Two.ENCODER,
        SwerveModuleConfigs.Two.DRIVE_MOTOR_REVERSED,
        SwerveModuleConfigs.Two.TURNING_MOTOR_REVERSED,
        SwerveModuleConfigs.Two.ABSOLUTE_ENCODER_REVERSED
    );

    private SwerveModule moduleThree = new SwerveModule(
        SwerveModuleConfigs.Three.DRIVE,
        SwerveModuleConfigs.Three.TURN,
        SwerveModuleConfigs.Three.ENCODER,
        SwerveModuleConfigs.Three.DRIVE_MOTOR_REVERSED,
        SwerveModuleConfigs.Three.TURNING_MOTOR_REVERSED,
        SwerveModuleConfigs.Three.ABSOLUTE_ENCODER_REVERSED
    ); 

    private SwerveModule moduleFour = new SwerveModule(
        SwerveModuleConfigs.Four.DRIVE,
        SwerveModuleConfigs.Four.TURN,
        SwerveModuleConfigs.Four.ENCODER,
        SwerveModuleConfigs.Four.DRIVE_MOTOR_REVERSED,
        SwerveModuleConfigs.Four.TURNING_MOTOR_REVERSED,
        SwerveModuleConfigs.Four.ABSOLUTE_ENCODER_REVERSED
    );

    private Pigeon2 gyro = new Pigeon2(RobotConstants.GYRO_ID, RobotConstants.SWERVE_CAN_BUS);

    // Initialized to starting position
    private SwerveDrivePoseEstimator odometer = new SwerveDrivePoseEstimator(
        SwerveKinematics.DRIVE_KINEMATICS, getRotation2d(), getModulePositions(),
        SwerveUtilities.getStartingPose(Telemetry.getInstance().getSelectedStartingPosition()));
    
    /** Keep track of desiredStates for {@link Telemetry} */
    private SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    /** Save whether or not currently using LL data for odometry */
    private boolean usingLimelightOdometry = false;
    
    /**
     * Creates a new SwerveSubsystem.
     */
    private SwerveSubsystem() {
        super("SwerveSubsystem");

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::setPose,
            this::getChassisSpeeds,
            this::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(4.5, 0, 0),
                new PIDConstants(4.5, 0, 0),
                SwerveKinematics.PHYSICAL_MAX_MODULE_SPEED,
                Math.sqrt(Math.pow(RobotConstants.WHEEL_BASE, 2) + Math.pow(RobotConstants.WHEEL_BASE, 2)) / 2,
                new ReplanningConfig()
            ),
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                Telemetry.logMessage("DriverStation alliance is not present", LoggingTags.ERROR);
                return false;
            },
            this
        );

        gyro.getConfigurator().apply(new MountPoseConfigs().withMountPoseYaw(0));
        
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                setHeading(SwerveUtilities.getStartingPose(Telemetry.getInstance().getSelectedStartingPosition())
                    .getRotation().getDegrees()
                );
            }
            catch (Exception error) {
                error.printStackTrace();
            }
        });
    }
    
    /**
     * Update the odometer and push ShuffleBoard data.
     */
    @Override
    public void periodic() {
        this.odometer.update(getRotation2d(), getModulePositions());
        
        this.usingLimelightOdometry = updateOdometryUsingVision();
    }

    /**
     * Sets the heading of the Pigeon2.
     * @param heading in degrees.
     */
    public void setHeading(double heading) {
        this.gyro.setYaw(heading);
        this.setPose(new Pose2d(getPose().getTranslation(), new Rotation2d(heading)));
    }

    /**
     * Zeros the heading of the Pigeon2.
     */
    public void setHeading() {
        setHeading(0);
    }
    
    /**
     * Gets the current heading of the robot.
     * @return heading in degrees.
     */
    public double getHeading() {
        return this.gyro.getYaw().getValueAsDouble();
    }

    /**
     * Zeros the positions of the drive encoders.
     */
    public void zeroDrivePositions() {
        this.moduleOne.zeroDriveEncoder();
        this.moduleTwo.zeroDriveEncoder();
        this.moduleThree.zeroDriveEncoder();
        this.moduleFour.zeroDriveEncoder();
    }

    /**
     * Gets the heading of the robot as a Rotation2d.
     * @return heading.
     */
    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    /**
     * Gets the current positions of the modules.
     * @return an array of the positions.
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            this.moduleOne.getPosition(),
            this.moduleTwo.getPosition(),
            this.moduleThree.getPosition(),
            this.moduleFour.getPosition()
        };
    }

    /**
     * Gets the current states of the modules.
     * @return an array of the states.
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
          this.moduleOne.getState(),
          this.moduleTwo.getState(),
          this.moduleThree.getState(),
          this.moduleFour.getState()
        };
    }

    /**
     * Gets the current position of the robot.
     * @return position as a Pose2d.
     */
    public Pose2d getPose() {
        return this.odometer.getEstimatedPosition();
    }
  
    /**
     * Resets the odometry of the robot.
     * @param pose to set to.
     */
    public void setPose(Pose2d pose) {
        this.odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }
  
    /**
     * Reset odometry translation to the position that the Limelight sees.
     * Does not reset rotation, which is tracked by the gyro.
     * @apiNote will not reset position if the Limelight is returning an empty position.
     */
    public void resetOdometryLimelight() {
        Translation2d translation = LimelightSubsystem.getInstance().getBotpose().getTranslation();
        if (!translation.equals(new Translation2d())) {
            setPose(new Pose2d(translation, getRotation2d()));
        }
    }

    /**
     * Get whether or not the odometry has recently updated using vision.
     * @return using Limelight odometry.
     */
    public boolean usingLimelightOdometry() {
        return this.usingLimelightOdometry;
    }

    /**
     * Calculates the necessary updates for the odometer.
     * @return whether or not it updated.
     * @see https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
     */
    private boolean updateOdometryUsingVision() {
        // Translation X, Y, Z.
        // Rotation Roll, Pitch, Yaw.
        // Total Latency (cl + tl).
        // Tag count, span, average distance, average area.
        double[] rawLimelightData = LimelightHelpers.getBotPose_wpiBlue(LimelightConstants.SHOOTER_LLIGHT);

        // No valid targets.
        if (rawLimelightData.length == 0 || rawLimelightData[0] == 0.0) return false;
        
        Pose2d limelightBotpose = new Pose2d(
            new Translation2d(rawLimelightData[0], rawLimelightData[1]),
            Rotation2d.fromDegrees(rawLimelightData[5])
        );
        double poseDifference = getPose().getTranslation().getDistance(limelightBotpose.getTranslation());

        // Any targets detected.
        if (rawLimelightData[7] > 0) {
            double xyStds;
            double degStds;
            
            // Two or more targets detected.
            if (rawLimelightData[7] >= 2) {
                xyStds = 0.5;
                degStds = 6;
            }
            // 1 target with large area and good accuracy.
            else if (rawLimelightData[10] > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            }
            // 1 far target but with good accuracy.
            else if (rawLimelightData[10] > 0.1 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 30;
            }
            // Not trustworthy enough.
            else {
                return false;
            }

            this.odometer.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStds, degStds, Units.degreesToRadians(degStds))
            );
            limelightBotpose = new Pose2d(
                limelightBotpose.getTranslation(),
                Rotation2d.fromDegrees(getHeading())
            );

            this.odometer.addVisionMeasurement(
                limelightBotpose, 
                Timer.getFPGATimestamp() - rawLimelightData[6] / 1000.0
            );

            return true;
        }
        return false;
    }

    /**
     * Stops all the swerve modules.
     */
    public void stopModules() {
        this.moduleOne.stop();
        this.moduleTwo.stop();
        this.moduleThree.stop();
        this.moduleFour.stop();
    }
    
    /**
     * Gets the current chassis speeds.
     * @return the chassis speeds.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return SwerveKinematics.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }
    
    /**
     * Converts the chassis speeds to module states and
     * sets them as the desired ones for the modules.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds correctedChasisSpeed = SwerveUtilities.correctForDynamics(chassisSpeeds);
        
        SwerveModuleState[] moduleStates = SwerveKinematics.DRIVE_KINEMATICS.toSwerveModuleStates(correctedChasisSpeed);
        this.setModuleStates(moduleStates);
    }

    /**
     * Sets the destired states to the correct swerve modules.
     * @param desiredStates the states to set.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
            SwerveKinematics.PHYSICAL_MAX_MODULE_SPEED);
        this.desiredStates = desiredStates;
        
        this.moduleOne.setDesiredState(desiredStates[0]);
        this.moduleTwo.setDesiredState(desiredStates[1]);
        this.moduleThree.setDesiredState(desiredStates[2]);
        this.moduleFour.setDesiredState(desiredStates[3]);
    }

    /**
     * Get desired states for use with {@link Telemetry}.
     * @return states.
     */
    public SwerveModuleState[] getDesiredStates() {
        return this.desiredStates;
    }
}
