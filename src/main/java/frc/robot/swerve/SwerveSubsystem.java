package frc.robot.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.ShuffleboardTabConstants;
import frc.robot.Constants.SwerveKinematics;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.limelight.LimelightSubsystem;
import frc.robot.utilities.Logger;
import frc.robot.utilities.SwerveUtilities;

public class SwerveSubsystem extends SubsystemBase {

    // Singleton Design Pattern
    private static SwerveSubsystem instance;
    public static SwerveSubsystem getInstance() {
        if(instance == null) {
            instance = new SwerveSubsystem();
        }
        return instance;
    }

    // Instance of swerve modules, initalized with specific value
    private SwerveModule moduleOne = new SwerveModule(
        SwerveModuleConstants.One.DRIVE,
        SwerveModuleConstants.One.TURN,
        SwerveModuleConstants.One.ENCODER,
        SwerveModuleConstants.One.DRIVE_MOTOR_REVERSED,
        SwerveModuleConstants.One.TURNING_MOTOR_REVERSED,
        SwerveModuleConstants.One.ABSOLUTE_ENCODER_REVERSED
    );

    private SwerveModule moduleTwo = new SwerveModule(
        SwerveModuleConstants.Two.DRIVE,
        SwerveModuleConstants.Two.TURN,
        SwerveModuleConstants.Two.ENCODER,
        SwerveModuleConstants.Two.DRIVE_MOTOR_REVERSED,
        SwerveModuleConstants.Two.TURNING_MOTOR_REVERSED,
        SwerveModuleConstants.Two.ABSOLUTE_ENCODER_REVERSED
    );

    private SwerveModule moduleThree = new SwerveModule(
        SwerveModuleConstants.Three.DRIVE,
        SwerveModuleConstants.Three.TURN,
        SwerveModuleConstants.Three.ENCODER,
        SwerveModuleConstants.Three.DRIVE_MOTOR_REVERSED,
        SwerveModuleConstants.Three.TURNING_MOTOR_REVERSED,
        SwerveModuleConstants.Three.ABSOLUTE_ENCODER_REVERSED
    ); 

    private SwerveModule moduleFour = new SwerveModule(
        SwerveModuleConstants.Four.DRIVE,
        SwerveModuleConstants.Four.TURN,
        SwerveModuleConstants.Four.ENCODER,
        SwerveModuleConstants.Four.DRIVE_MOTOR_REVERSED,
        SwerveModuleConstants.Four.TURNING_MOTOR_REVERSED,
        SwerveModuleConstants.Four.ABSOLUTE_ENCODER_REVERSED
    );

    // Instance of the Pigeon2 gyroscope on the specifc swerve CAN bus
    private Pigeon2 gyro = new Pigeon2(SwerveModuleConstants.GRYO_ID, SwerveModuleConstants.SWERVE_CAN_BUS);

    // Instance of the odometer to track robot position, initialized to starting position
    private SwerveDrivePoseEstimator odometer = new SwerveDrivePoseEstimator(
        SwerveKinematics.DRIVE_KINEMATICS, getRotation2d(), getModulePositions(), SwerveUtilities.getStartingPosition());
    
    // Initialize a field to track of robot position in SmartDashboard
    private Field2d swerve_field = new Field2d();

    // Shuffleboard
    private GenericEntry SB_GYRO = Shuffleboard.getTab(ShuffleboardTabConstants.DEFAULT)
        .add("Robot Heading", 0)
        .withWidget(BuiltInWidgets.kGyro)
        .withPosition(0, 0)
        .withSize(3, 3)
        .getEntry();
    
    private Logger logger;
    private SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    /**
    * Initializes a new SwerveSubsystem object, configures PathPlannerLib AutoBuilder,
    * and zeros the heading after a delay to allow the pigeon to turn on and load
    */
    private SwerveSubsystem() {
        this.logger = new Logger();

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getChassisSpeeds,
            this::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(4.5, 0, 0),
                new PIDConstants(3, 0, 0),
                SwerveKinematics.PHYSICAL_MAX_MODULE_SPEED, // might need to change to an auton constant
                Math.sqrt(Math.pow(PhysicalConstants.WHEEL_BASE, 2) + Math.pow(PhysicalConstants.WHEEL_BASE, 2)) / 2,
                new ReplanningConfig()),
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this);
        
        // Set up custom logging to add the current path to a field 2d widget on shuffleboard
        PathPlannerLogging.setLogActivePathCallback((poses) -> swerve_field.getObject("path").setPoses(poses));
        Shuffleboard.getTab(ShuffleboardTabConstants.FIELDS)
            .add("Field (SwervePoseEstimator)", swerve_field)
            .withWidget(BuiltInWidgets.kField)
            .withPosition(0, 0)
            .withSize(7, 4);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                setHeading(SwerveUtilities.getStartingPosition().getRotation().getDegrees());
            }
            catch (Exception error) {
                error.printStackTrace();
            }});
    }

    /**
    * Resets the heading of the robot's Piegon2
    *
    * @param heading in degrees
    */
    public void setHeading(double heading) {
        this.gyro.setYaw(heading);
        this.resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d(heading)));
    }

    /**
     * Zeros the heading of the Pigeon2
     */
    public void zeroHeading() {
        setHeading(0);
    }
    /**
     * Zeros the position of the drive encoders
     */
    public void zeroDrivePositions() {
        this.moduleOne.zeroDriveEncoder();
        this.moduleTwo.zeroDriveEncoder();
        this.moduleThree.zeroDriveEncoder();
        this.moduleFour.zeroDriveEncoder();
    }

    /**
    * Returns the current heading of the robot in degrees
    * 
    * @return current heading of the robot
    */
    public double getHeading() {
        return this.gyro.getYaw().getValueAsDouble();
    }

    /**
    * Returns the current rotation information of the robot
    * 
    * @return current rotation of the robot
    */
    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    /**
    * Returns the current positions of the modules
    *
    * @return array of the positions
    */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[] {
            this.moduleOne.getPosition(),
            this.moduleTwo.getPosition(),
            this.moduleThree.getPosition(),
            this.moduleFour.getPosition()};
        return positions;
    }

    /**
    * Returns the current states of the modules
    *
    * @return array of the states
    */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[] {
          this.moduleOne.getState(),
          this.moduleTwo.getState(),
          this.moduleThree.getState(),
          this.moduleFour.getState()};
        return states;
    }

    /**
    * Gets the current pose of the robot in meters
    *
    * @return the pose of the robot in meters
    */
    public Pose2d getPose() {
        return this.odometer.getEstimatedPosition();
    }
  
    /**
    * Resets the odometry of the robot
    */
    public void resetOdometry(Pose2d pose) {
        this.odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }
  
    /**
    * Update the odometer and push ShuffleBoard data
    */
    @Override
    public void periodic() {
        this.odometer.update(getRotation2d(), getModulePositions());
        this.logger.execute();
        
        this.updateOdometryUsingVision();

        this.swerve_field.setRobotPose(getPose());
        this.SB_GYRO.setDouble(getHeading());
    }

    /**
     * Calculates the necessary updates for the odometer
     */
    private void updateOdometryUsingVision() {
        if (!LimelightSubsystem.getInstance().hasTarget(LimelightConstants.FRONT_LIMELIGHT)) return;

        Pose2d botpose = LimelightSubsystem.getInstance().getBotpose();
        Pose2d relative = botpose.relativeTo(getPose());

        if (Math.abs(relative.getX()) <= LimelightConstants.ODOMETRY_ALLOWED_ERROR_METERS[0]
            && Math.abs(relative.getY()) <= LimelightConstants.ODOMETRY_ALLOWED_ERROR_METERS[1]) {
            this.odometer.addVisionMeasurement(
                botpose, Timer.getFPGATimestamp()
                - LimelightSubsystem.getInstance().getLatency(LimelightConstants.FRONT_LIMELIGHT));
        }
    }

    /**
    * Stops all the swerve modules
    */
    public void stopModules() {
        this.moduleOne.stop();
        this.moduleTwo.stop();
        this.moduleThree.stop();
        this.moduleFour.stop();
    }
    
    /** Gets the Chassis speeds
     *
     * @return the chassis speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return SwerveKinematics.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }
    
    /**
     * Converts the chassis speeds to module states and
     * sets them as the desired ones for the modules
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds correctedChasisSpeed = SwerveUtilities.correctForDynamics(chassisSpeeds);
        // ChassisSpeeds correctedChasisSpeed = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] moduleStates = SwerveKinematics.DRIVE_KINEMATICS.toSwerveModuleStates(correctedChasisSpeed);
        this.setModuleStates(moduleStates);
    }

    /**
     * Sets the destired states to the correct swerve modules
     * 
     * @param desiredStates - states to be relayed to the swerve modules
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
     * Get desired states 
     * 
     * @return states
     */
    public SwerveModuleState[] getDesiredStates() {
        return this.desiredStates;
    }
}
