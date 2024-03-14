// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {

    // Singleton Design Pattern
    private static LimelightSubsystem instance;

    public static LimelightSubsystem getInstance() {
        if (instance == null) {
            instance = new LimelightSubsystem();
        }
        return instance;
    }

    // Field2d limelight_field = new Field2d();

    /** Creates a new LimelightSubsystem. */
    public LimelightSubsystem() {
        super("LimelightSubsystem");
        

        // Shuffleboard.getTab(ShuffleboardTabConstants.FIELDS)
        //     .add("Field (limelight)", limelight_field)
        //     .withWidget(BuiltInWidgets.kField)
        //     .withPosition(7, 0)
        //     .withSize(7, 4);
    }

    /**
     * Horizontal Offset From Crosshair To Target
     *
     * @return offset
     */
    public double getHorizontalOffset(String limelight) {
        return LimelightHelpers.getTX(limelight);
    }

    /**
     * Vertical Offset From Crosshair To Target
     *
     * @return offset
     */
    public double getVerticalOffset(String limelight) {
        return LimelightHelpers.getTY(limelight);
    }

    /**
     * Target Area (0% of image to 100% of image)
     * (uses LimelightConstants.BACK_LIMELIGHT)
     *
     * @return area
     */
    public double getTargetArea() {
        return LimelightHelpers.getTA(LimelightConstants.INTAKE_LLIGHT);
    }

    /**
     * Wheter the Limelight has a target
     *
     * @return offset
     */
    public boolean hasTarget(String limelight) {
        return LimelightHelpers.getTV(limelight);
    }

    /**
     * Gets the ID of the nearest AprilTag or 0 if not found
     * (using LimelightConstants.FRONT_LIMELIGHT)
     *
     * @return ID
     */
    public int getTargetID() {
        return (int) NetworkTableInstance.getDefault().getTable(LimelightConstants.SHOOTER_LLIGHT)
            .getEntry("tid").getInteger(0);
    }

    /**
     * Gets the botpose relative to the current blue alliance
     * (using LimelightConstants.FRONT_LIMELIGHT)
     *
     * @return botpose
     */
    public Pose2d getBotpose() {
        // return new Pose2d();
        return LimelightHelpers.getBotPose2d_wpiBlue(LimelightConstants.SHOOTER_LLIGHT);
    }

    /**
     * Gets the amount of April Tags in view
     * (using LimelightConstants.SHOOTER_LLight)
     * 
     * @return amount of April Tags in view
     */
    public int getVisibleTags() {
        return (int) LimelightHelpers.getBotPose_wpiBlue(LimelightConstants.SHOOTER_LLIGHT)[7];
    }

    /**
     * Gets the latency of the limelight to be used for odometry
     * 
     * @return latency in seconds
     */
    public double getLatency(String limelight) {
        return LimelightHelpers.getLatency_Pipeline(limelight) / 1000
                - LimelightHelpers.getLatency_Capture(limelight) / 1000;
    }

    @Override
    public void periodic() {
        // limelight_field.setRobotPose(this.getBotpose());
        // int tid = this.getTargetID();
        // boolean tSee = this.hasTarget(LimelightConstants.SHOOTER_LLIGHT);
        // boolean nSee = this.hasTarget(LimelightConstants.INTAKE_LLIGHT);

        // Default Shuffleboard
        // SB_D_TSEE.setBoolean(tSee);
        // SB_D_NSEE.setBoolean(nSee);

        // Field Shuffleboard (disabled for competition)
        // SB_F_TID.setInteger(tid);
        // SB_F_TSEE.setBoolean(tSee);
    }
}
