package frc.robot.utilities;

import java.util.Optional;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.constants.Positions;
import frc.robot.constants.Constants.TelemetryConstants.LoggingTags;
import frc.robot.constants.Positions.StartingPositions;

public class SwerveUtilities {
    
    /**
     * Team 254 implementation to fix Swerve Drive skew.
     * @param transform
     * @return no idea.
     * @see https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/8.
     */
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

    /**
     * Team 254 implementation to fix Swerve Drive skew .
     * @param originalSpeeds Original Chasis speeds.
     * @return Corrected chasis speeds.
     * @see https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/8.
     */
    public static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = Robot.kDefaultPeriod;
        Pose2d futureRobotPose = new Pose2d(
            originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
            originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
            Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = SwerveUtilities.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds = new ChassisSpeeds(
            twistForPose.dx / LOOP_TIME_S,
            twistForPose.dy / LOOP_TIME_S,
            twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
    }

    /**
     * Get the starting position of the robot.
     * @param startingPosition override the alliance position-based starting position.
     * @return the starting position.
     */
    public static Pose2d getStartingPose(StartingPositions startingPosition) {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            Telemetry.logMessage("DriverStation alliance is not present", LoggingTags.ERROR);
            alliance = Optional.ofNullable(DriverStation.Alliance.Blue);
        }
        OptionalInt location = startingPosition.equals(StartingPositions.AUTO)
            ? DriverStation.getLocation() : OptionalInt.of(startingPosition.getLocation());
        
        Pose2d pose;
        if (!alliance.isPresent()) {
            pose = new Pose2d();
        }
        else if (!startingPosition.equals(StartingPositions.AUTO)) {
            pose = Positions.STARTING_POSITIONS.get(alliance.get()).get(startingPosition.getLocation());
        }
        else if (location.isPresent()) {
            pose = Positions.STARTING_POSITIONS.get(alliance.get()).get(location.getAsInt());
        }
        else {
            pose = new Pose2d();
        }
        return pose;
    }
}
