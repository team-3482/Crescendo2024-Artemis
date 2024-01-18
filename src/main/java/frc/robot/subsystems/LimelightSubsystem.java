// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {
    private double xPos;
    private double yPos;
    private double angle;
    private Timer timeout;

    /** Creates a new ExampleSubsystem. */
    public LimelightSubsystem() {
        this.timeout = new Timer();
    }
    
    /**
     * Gets the x-axis position of the nearest AprilTag
     *
     * @return x-axis position
     */
    public double getXPos() {
        return this.xPos;
    }
    
    /**
     * Gets the y-axis position of the nearest AprilTag
     *
     * @return y-axis position
     */
    public double getYPos() {
        return this.yPos;
    }

    /**
     * Gets the angle for the nearest AprilTag relative to TX
     *
     * @return angle 
     */
    public double getAngle() {
        return this.angle;
    }

    @Override
    public void periodic() {
        final double TX = LimelightHelpers.getTX("limelight");
        final double TY = LimelightHelpers.getTY("limelight");
        // Angle of AprilTag relative to TX
        final double calculated_angle = Units.radiansToDegrees(Math.tan(TY / TX));

        // The limelight sets tx/ty to 0.0 if no AprilTag is found.
        // This prevents our robot from getting incorrect position data and getting confused

        // The timeout makes sure that the positions are reset to 0 if there is no AprilTag
        // found for at least LimelightConstants.TIMEOUT time, so we can later check
        // for 0 and the robot will not pathfind to outdated positions
        if (TX == 0 && TY == 0) {
            if (timeout.hasElapsed(LimelightConstants.TIMEOUT)) {
                this.xPos = 0;
                this.yPos = 0;
                this.angle = 0;
            }
            else if (timeout.get() == 0) {
                timeout.restart(); // Sets timer to 0 and starts it
            }
        }
        else {
            this.xPos = TX;
            this.xPos = TY;
            this.angle = calculated_angle;
            timeout.stop();
            timeout.reset(); // Sets timer to 0
        }

        SmartDashboard.putNumber("Limelight TX", this.xPos);
        SmartDashboard.putNumber("Limelight TY", this.yPos);
        SmartDashboard.putNumber("Limelight TA", this.angle);
        SmartDashboard.putNumber("Limeliht Timeout (s)", this.timeout.get());
    }
}

