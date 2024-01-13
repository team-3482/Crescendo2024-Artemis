// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

/** An example command that uses an example subsystem. */
public class Limelight extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public double x;
  public double y;

  public Limelight() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double tx = LimelightHelpers.getTX("limelight");
    final double ty = LimelightHelpers.getTY("limelight");
    
    // limelight sets tx/ty to 0.0 if no AprilTag is found.
    // this prevents our robot from getting incorrect position data and getting confused
    if (tx != 0.0 && ty != 0.0){
      x = tx;
      y = ty;
    }

    SmartDashboard.putNumber("Limelight TX", tx);
    SmartDashboard.putNumber("Limelight TY", ty);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
