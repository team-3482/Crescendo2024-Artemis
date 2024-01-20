package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonConstants;
// import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.SwerveKinematics;
import frc.robot.Constants.SwerveModuleConstants;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class SwerveSubsystem extends SubsystemBase {
    // Instance of swerve modules, initalized with specific value
    private SwerveModule moduleOne = new SwerveModule(
        SwerveModuleConstants.One.DRIVE,
        SwerveModuleConstants.One.TURN,
        SwerveModuleConstants.One.ENCODER,
        SwerveModuleConstants.One.DRIVE_MOTOR_REVERSED,
        SwerveModuleConstants.One.TURNING_MOTOR_REVERSED,
        SwerveModuleConstants.One.ENCODER_OFFSET_ROT,
        SwerveModuleConstants.One.ABSOLUTE_ENCODER_REVERSED
    );

    private SwerveModule moduleTwo = new SwerveModule(
        SwerveModuleConstants.Two.DRIVE,
        SwerveModuleConstants.Two.TURN,
        SwerveModuleConstants.Two.ENCODER,
        SwerveModuleConstants.Two.DRIVE_MOTOR_REVERSED,
        SwerveModuleConstants.Two.TURNING_MOTOR_REVERSED,
        SwerveModuleConstants.Two.ENCODER_OFFSET_ROT,
        SwerveModuleConstants.Two.ABSOLUTE_ENCODER_REVERSED
    );

    private SwerveModule moduleThree = new SwerveModule(
        SwerveModuleConstants.Three.DRIVE,
        SwerveModuleConstants.Three.TURN,
        SwerveModuleConstants.Three.ENCODER,
        SwerveModuleConstants.Three.DRIVE_MOTOR_REVERSED,
        SwerveModuleConstants.Three.TURNING_MOTOR_REVERSED,
        SwerveModuleConstants.Three.ENCODER_OFFSET_ROT,
        SwerveModuleConstants.Three.ABSOLUTE_ENCODER_REVERSED
    );

    private SwerveModule moduleFour = new SwerveModule(
        SwerveModuleConstants.Four.DRIVE,
        SwerveModuleConstants.Four.TURN,
        SwerveModuleConstants.Four.ENCODER,
        SwerveModuleConstants.Four.DRIVE_MOTOR_REVERSED,
        SwerveModuleConstants.Four.TURNING_MOTOR_REVERSED,
        SwerveModuleConstants.Four.ENCODER_OFFSET_ROT,
        SwerveModuleConstants.Four.ABSOLUTE_ENCODER_REVERSED
    );

    // Instance of the Pigeon2 gyroscope on the specifc swerve CAN bus
    private Pigeon2 gyro = new Pigeon2(SwerveModuleConstants.GRYO_ID, SwerveModuleConstants.SWERVE_CAN_BUS);

    private SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        SwerveKinematics.driveKinematics,
        new Rotation2d(0), getModulePositions());
    
    private Field2d field = new Field2d();

    /**
    * Initializes a new SwerveSubsystem object, configures PathPlannerLib AutoBuilder,
    * and zeros the heading after a delay to allow the pigeon to turn on and load
    */
    public SwerveSubsystem() {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getChassisSpeeds,
            this::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(20, 20, 0),
                new PIDConstants(20, 20, 0),
                AutonConstants.MAX_DRIVE_SPEED_METERS_PER_SECOND_AUTON,
                PhysicalConstants.WHEEL_BASE / 2,
                new ReplanningConfig()),
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this);
        
        // Set up custom logging to add the current path to a field 2d widget
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
        SmartDashboard.putData("Field", field);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            }
            catch (Exception error) {
                error.printStackTrace();
            }});
    }

    /**
    * Zeros the heading of the robot Pigeon2
    */
    public void zeroHeading() {
        gyro.setYaw(0);
        // this.resetOdometry(getPose());
    }

    /**
     * Zeros the position of the drive encoders
     */
    public void zeroDrivePositions() {
        this.moduleOne.zeroDriveEncoder();
        this.moduleTwo.zeroDriveEncoder();
        this.moduleThree.zeroDriveEncoder();
        this.moduleFour.zeroDriveEncoder();
        // this.resetOdometry(getPose());
    }

    /**
    * Returns the current heading of the robot
    * 
    * @return current heading of the robot
    */
    public double getHeading() {
        return gyro.getYaw().getValueAsDouble();
    }

    /**
    * Returns the current rotation information of the robot
    * 
    * @return current rotation of the robot
    */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
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
        return odometer.getPoseMeters();
    }
  
    /**
    * Resets the odometry of the robot
    */
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }
  
    /**
    * Update the odometer and push SmartDashboard data
    */
    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());
        field.setRobotPose(getPose());

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location (odometer)", getPose().getTranslation().toString());
        SmartDashboard.putString("Robot Rotation (odometer)", getPose().getRotation().toString());
    }

    public static Twist2d log(Pose2d transform) {
        final double kEps = 1E-9;

        final double dtheta = transform.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
        double halftheta_by_tan_of_halfdtheta;

        if (Math.abs(cos_minus_one) < kEps) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        }
        else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;
        }

        final Translation2d translation_part = transform.getTranslation()
                .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
        return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
    }

    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02;
        Pose2d futureRobotPose = new Pose2d(
            originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
            originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
            Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = log(futureRobotPose);
        ChassisSpeeds updatedSpeeds = new ChassisSpeeds(
            twistForPose.dx / LOOP_TIME_S,
            twistForPose.dy / LOOP_TIME_S,
            twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
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
        return SwerveKinematics.driveKinematics.toChassisSpeeds(getModuleStates());
    }
    
    /**
     * Converts the chassis speeds to module states and
     * sets them as the desired ones for the modules
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds correctedChasisSpeed = correctForDynamics(chassisSpeeds);
        SwerveModuleState[] moduleStates = SwerveKinematics.driveKinematics.toSwerveModuleStates(correctedChasisSpeed);
        setModuleStates(moduleStates);
    }

    /**
     * Sets the destired states to the correct swerve modules
     * 
     * @param desiredStates - states to be relayed to the swerve modules
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
            SwerveKinematics.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);

        this.moduleOne.setDesiredState(desiredStates[0]);
        this.moduleTwo.setDesiredState(desiredStates[1]);
        this.moduleThree.setDesiredState(desiredStates[2]);
        this.moduleFour.setDesiredState(desiredStates[3]);
    }

    /**
     * Ouputs information of the current swerve system
     */
    public void outputEncoderValues() {
        this.moduleOne.outputEncoderPosition();
        this.moduleTwo.outputEncoderPosition();
        this.moduleThree.outputEncoderPosition();
        this.moduleFour.outputEncoderPosition();
    }
}
