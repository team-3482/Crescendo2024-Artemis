// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
    private double x;
    private double y;

    /** Creates a new ExampleSubsystem. */
    public LimelightSubsystem() {}

    @Override
    public void periodic() {
        final double tx = LimelightHelpers.getTX("limelight");
        final double ty = LimelightHelpers.getTY("limelight");

        // limelight sets tx/ty to 0.0 if no AprilTag is found.
        // this prevents our robot from getting incorrect position data and getting confused
        if (tx != 0.0 && ty != 0.0){
            x = tx;
            y = ty;
        }

        SmartDashboard.putNumber("Limelight TX", x);
        SmartDashboard.putNumber("Limelight TY", y);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}

