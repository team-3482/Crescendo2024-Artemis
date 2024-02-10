package frc.robot.Utilities;

import java.util.Optional;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AutonConstants;

public class SwerveUtilities {
    
    /**
     * Team 254 implementation to fix Swerve Drive skew 
     * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/8
     *
     * @param transform
     * @return
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
     * Team 254 implementation to fix Swerve Drive skew 
     * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/8
     * 
     * @param originalSpeeds - Original Chasis speeds
     * @return Corrected chasis speeds
     */
    public static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02;
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
     * Grab the starting position of the robot
     * 
     * @return the starting position
     */
    public static Pose2d getStartingPosition() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        OptionalInt location = DriverStation.getLocation();
        Pose2d startingPosition;
        if (!location.isPresent() || !alliance.isPresent()) {
            startingPosition = new Pose2d();
        }
        else {
            startingPosition = AutonConstants.STARTING_POSITIONS.get(alliance.get()).get(location.getAsInt());
        }
        return startingPosition;
    }
}
