// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
    public double x;
    public double y;

    /** Creates a new ExampleSubsystem. */
    public LimelightSubsystem() {}

    /**
    * Example command factory method.
    *
    * @return a command
    */
    public Command printXY() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                SmartDashboard.putNumber("Limelight TX", x);
                SmartDashboard.putNumber("Limelight TY", y);
            });
    }

    /**
    * An example method querying a boolean state of the subsystem (for example, a
    * digital sensor).
    *
    * @return value of some boolean subsystem state, such as a digital sensor.
    */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

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


    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
