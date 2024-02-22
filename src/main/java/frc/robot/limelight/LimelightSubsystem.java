// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.limelight;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShuffleboardTabConstants;

public class LimelightSubsystem extends SubsystemBase {

  // Singleton Design Pattern
  private static LimelightSubsystem instance;

  public static LimelightSubsystem getInstance() {
    if (instance == null) {
      instance = new LimelightSubsystem();
    }
    return instance;
  }

  Field2d limelight_field = new Field2d();

  // Shuffleboard
  private GenericEntry SB_D_TID = Shuffleboard.getTab(ShuffleboardTabConstants.DEFAULT)
      .add("T-ID", 0)
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(3, 3)
      .withSize(1, 1)
      .getEntry();
  private GenericEntry SB_D_NSEE = Shuffleboard.getTab(ShuffleboardTabConstants.DEFAULT)
      .add("NOTE-SEE", false)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withPosition(3, 0)
      .withSize(3, 3)
      .getEntry();
    private GenericEntry SB_D_TSEE = Shuffleboard.getTab(ShuffleboardTabConstants.DEFAULT)
        .add("TAG-SEE", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(6, 0)
        .withSize(3, 3)
        .getEntry();
    private GenericEntry SB_D_TUPDATE = Shuffleboard.getTab(ShuffleboardTabConstants.DEFAULT)
        .add("ADD-VISION", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(9, 0)
        .withSize(3, 3)
        .getEntry();

  private GenericEntry SB_F_TID = Shuffleboard.getTab(ShuffleboardTabConstants.FIELDS)
      .add("T-ID", 0)
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(14, 4)
      .withSize(2, 1)
      .getEntry();
  private GenericEntry SB_F_TSEE = Shuffleboard.getTab(ShuffleboardTabConstants.FIELDS)
      .add("TAG-SEE", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(14, 0)
        .withSize(2, 2)
        .getEntry();
    private GenericEntry SB_F_TUPDATE = Shuffleboard.getTab(ShuffleboardTabConstants.FIELDS)
        .add("ADD-VISION", false)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withPosition(14, 2)
      .withSize(2, 2)
      .getEntry();

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    final HttpCamera backLimelightFeed = new HttpCamera(
        "front limelight",
        "http://10.34.82.3:5800/stream.mjpg",
        HttpCameraKind.kMJPGStreamer);

    final MjpegServer backLimelight = CameraServer.startAutomaticCapture(backLimelightFeed);

    Shuffleboard.getTab(ShuffleboardTabConstants.FIELDS)
        .add("Field (limelight)", limelight_field)
        .withWidget(BuiltInWidgets.kField)
        .withPosition(7, 0)
        .withSize(7, 4);

    Shuffleboard.getTab(ShuffleboardTabConstants.FIELDS)
        .add("back limelight view", backLimelight)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(0, 0)
        .withSize(4, 3);

  }

  /**
   * Horizontal Offset From Crosshair To Target
   * (uses LimelightConstants.BACK_LIMELIGHT)
   *
   * @return offset
   */
  public double getHorizontalOffset() {
    return LimelightHelpers.getTX(LimelightConstants.BACK_LIMELIGHT);
  }

  /**
   * Vertical Offset From Crosshair To Target
   * (uses LimelightConstants.BACK_LIMELIGHT)
   *
   * @return offset
   */
  public double getVerticalOffset() {
    return LimelightHelpers.getTY(LimelightConstants.BACK_LIMELIGHT);
  }

  /**
   * Target Area (0% of image to 100% of image)
   * (uses LimelightConstants.BACK_LIMELIGHT)
   *
   * @return area
   */
  public double getTargetArea() {
    return LimelightHelpers.getTA(LimelightConstants.BACK_LIMELIGHT);
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
  public int getID() {
    return (int) NetworkTableInstance.getDefault().getTable(LimelightConstants.FRONT_LIMELIGHT)
        .getEntry("tid").getInteger(0);
  }

  /**
   * Gets the botpose relative to the current blue alliance
   * (using LimelightConstants.FRONT_LIMELIGHT)
   *
   * @return botpose
   */
  public Pose2d getBotpose() {
    return LimelightHelpers.getBotPose2d_wpiBlue(LimelightConstants.FRONT_LIMELIGHT);
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

    // /**
    //  * Gets the position of an object being targeted
    //  * (uses LimelightConstants.BACK_LIMELIGHT)
    //  * 
    //  * @return the translation from the bot
    //  */
    // public Optional<Translation2d> 
    
    /**
     * Update the ADD-VISION Shuffleboard entries
     * 
     * @param bool whether or not it is updating odometry using vision
     */
    public void updateAddVisionEntry(boolean bool) {
        SB_D_TUPDATE.setBoolean(bool);
        SB_F_TUPDATE.setBoolean(bool);
    }

    @Override
    public void periodic() {
        limelight_field.setRobotPose(this.getBotpose());
        int tid = this.getID();
        boolean tSee = this.hasTarget(LimelightConstants.FRONT_LIMELIGHT);
        boolean nSee = this.hasTarget(LimelightConstants.BACK_LIMELIGHT);
        
        // Default Shuffleboard
        SB_D_TID.setInteger(tid);
        SB_D_TSEE.setBoolean(tSee);
        SB_D_NSEE.setBoolean(nSee);

        // Field Shuffleboard
        SB_F_TID.setInteger(tid);
        SB_F_TSEE.setBoolean(tSee);
    }
}
