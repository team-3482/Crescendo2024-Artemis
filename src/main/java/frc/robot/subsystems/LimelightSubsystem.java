// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {
    Field2d limelight_field = new Field2d();

    /** Creates a new LimelightSubsystem. */
    public LimelightSubsystem() {
        SmartDashboard.putData("Field (limelight)", limelight_field);
    }
    
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
     * Gets the ID of the nearest AprilTag or 0 if not found
     *
     * @return ID
     */
    public int getID() {
        return (int) NetworkTableInstance.getDefault().getTable(LimelightConstants.FRONT_LIMELIGHT)
            .getEntry("tid").getInteger(0);
    }

    /**
     * Gets the botpose relative to the current alliance, or Blue
     * if no alliance is found
     *
     * @return botpose
     */
    public Pose2d getBotpose() {
        // var alliance = DriverStation.getAlliance();
        // if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        //     return LimelightHelpers.getBotPose2d_wpiRed(LimelightConstants.FRONT_LIMELIGHT);
        // }
        return LimelightHelpers.getBotPose2d_wpiBlue(LimelightConstants.FRONT_LIMELIGHT);
    }

    /**
     * Gets the botpose in target space array of 6
     * 
     * @return botpose array
     */
    public double[] getBotPoseTargetSpace() {
        return LimelightHelpers.getBotPose_TargetSpace(LimelightConstants.FRONT_LIMELIGHT);
    }

    @Override
    public void periodic() {
        limelight_field.setRobotPose(this.getBotpose());

        // SmartDashboard.putString("Robot Location (limelight)", this.getBotpose().getTranslation().toString());
        // SmartDashboard.putString("Robot Rotation (limelight)", this.getBotpose().getRotation().toString());
        SmartDashboard.putNumber("T-ID", this.getID());
    }

    // private class ExtraHelpers extends LimelightHelpers {
    //     public static Pose2d toPose2d(double[] inData) {
    //         // From LimelightHelpers 397-406
    //         if(inData.length < 6) {
    //             System.err.println("Bad LL 2D Pose Data!");
    //             return new Pose2d();
    //         }
    //         Translation2d tran2d = new Translation2d(inData[0], inData[1]);
    //         Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
    //         return new Pose2d(tran2d, r2d);
    //     }
    // }
}
