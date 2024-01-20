// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {
    /** Creates a new LimelightSubsystem. */
    public LimelightSubsystem() {}
    
    /**
     * Horizontal Offset From Crosshair To Target
     *
     * @return offset
     */
    public double getTX() {
        return LimelightHelpers.getTX(LimelightConstants.FRONT_LIMELIGHT);
    }
    
    /**
     * Vertical Offset From Crosshair To Target
     *
     * @return offset
     */
    public double getTY() {
        return LimelightHelpers.getTY(LimelightConstants.FRONT_LIMELIGHT);
    }

    /**
     * Gets the angle for the nearest AprilTag relative to TX
     *
     * @return angle (in degrees)
     */
    public double getAngle() {
        // Need to use TX and calculate the angle for the robot to turn to so that TX 
        // is as close to 0 as possible (distance from crosshair to center of target)
        return Units.radiansToDegrees(0); 
    }
    
    /**
     * Gets the ID of the nearest AprilTag
     *
     * @return ID
     */
    public int getID() {
        return (int) NetworkTableInstance.getDefault().getTable(LimelightConstants.FRONT_LIMELIGHT)
            .getEntry("tid").getInteger(-1);
    }

    /**
     * Gets the botpose relative to the current alliance, or Blue
     * if no alliance is found
     *
     * @return botpose
     */
    public Pose2d getBotpose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return LimelightHelpers.getBotPose2d_wpiRed(LimelightConstants.FRONT_LIMELIGHT);
        }
        return LimelightHelpers.getBotPose2d_wpiBlue(LimelightConstants.FRONT_LIMELIGHT);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("AprilTag TX", this.getTX());
        SmartDashboard.putNumber("AprilTag TY", this.getTY());

        SmartDashboard.putString("Robot Location (limelight)", this.getBotpose().getTranslation().toString());
        SmartDashboard.putString("Robot Rotation (limelight)", this.getBotpose().getRotation().toString());
    }

    private class ExtraHelpers extends LimelightHelpers {
        public static Pose2d toPose2d(double[] inData) {
            // From LimelightHelpers 397-406
            if(inData.length < 6) {
                System.err.println("Bad LL 2D Pose Data!");
                return new Pose2d();
            }
            Translation2d tran2d = new Translation2d(inData[0], inData[1]);
            Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
            return new Pose2d(tran2d, r2d);
        }
    }
}
