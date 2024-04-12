// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.limelight;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.TelemetryConstants.LoggingTags;
import frc.robot.constants.PhysicalConstants.LimelightConstants;
import frc.robot.utilities.Telemetry;

/**
 * A subsystem that is mainly used to get data from Limelights.
 */
public class LimelightSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile LimelightSubsystem instance;
    private static Object mutex = new Object();

    public static LimelightSubsystem getInstance() {
        LimelightSubsystem result = instance;
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new LimelightSubsystem();
                }
            }
        }
        return instance;
    }

    /**
     * Creates a new LimelightSubsystem.
     */
    public LimelightSubsystem() {
        super("LimelightSubsystem");
        
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            Telemetry.logMessage("DriverStation alliance is not present", LoggingTags.ERROR);
            return;
        }

        // Target 7 when blue or when no alliance is found, and 4 for red side.
        LimelightHelpers.getLimelightNTTableEntry(LimelightConstants.SHOOTER_LLIGHT, "priorityid")
            .setInteger(alliance.isPresent() && alliance.get() == Alliance.Red ? 4 : 7);
    }

    @Override
    public void periodic() {}

    /**
     * Horizontal offset from crosshair to target.
     * @return offset in degrees.
     */
    public double getHorizontalOffset(String limelight) {
        return LimelightHelpers.getTX(limelight);
    }

    /**
     * Vertical offset from crosshair to target.
     * @return offset in degrees.
     */
    public double getVerticalOffset(String limelight) {
        return LimelightHelpers.getTY(limelight);
    }

    /**
     * Whether the Limelight has a target.
     * @return has a target.
     */
    public boolean hasTarget(String limelight) {
        return LimelightHelpers.getTV(limelight);
    }

    /**
     * Gets the ID of the nearest AprilTag or 0 if not found.
     * @return the ID.
     * @apiNote Uses {@link LimelightConstants#SHOOTER_LLIGHT}.
     */
    public int getTargetID() {
        return (int) NetworkTableInstance.getDefault().getTable(LimelightConstants.SHOOTER_LLIGHT)
            .getEntry("tid").getInteger(0);
    }

    /**
     * Gets the botpose relative to the blue alliance.
     * @return botpose.
     */
    public Pose2d getBotpose() {
        return LimelightHelpers.getBotPose2d_wpiBlue(LimelightConstants.SHOOTER_LLIGHT);
    }

    /**
     * Gets the amount of AprilTags in view.
     * @return amount.
     * @apiNote Uses {@link LimelightConstants#SHOOTER_LLIGHT}.
     */
    public int getVisibleTags() {
        return (int) LimelightHelpers.getBotPose_wpiBlue(LimelightConstants.SHOOTER_LLIGHT)[7];
    }

    /**
     * Gets the latency of the limelight.
     * @return latency in seconds.
     */
    public double getLatency(String limelight) {
        return LimelightHelpers.getLatency_Pipeline(limelight) / 1000
            - LimelightHelpers.getLatency_Capture(limelight) / 1000;
    }
}
