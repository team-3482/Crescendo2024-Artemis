// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OrbitConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.swerve.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class PivotShooterCommand extends Command {
    private double shootingAngle;

    /**
    * Creates a new PivotShooterCommand.
    */
    public PivotShooterCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem.getInstance());
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        LEDSubsystem.getInstance().setLightState(LightState.CMD_INIT);
        // Double so it can be null if the ID cannot be orbited
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            ShooterSubsystem.getInstance().canShoot = false;
            end(true);
            return;
        }
        ShooterSubsystem.getInstance().canShoot = true;

        Translation3d point = OrbitConstants.ORBIT_POINT.get(DriverStation.getAlliance().get());
        Pose2d botpose = SwerveSubsystem.getInstance().getPose();
        
        double dist = Math.sqrt(
            Math.pow(point.getX() - botpose.getX(), 2) + 
            Math.pow(point.getY() - botpose.getY(), 2)
        );
        this.shootingAngle = Math.atan((point.getZ() - PhysicalConstants.SHOOTER_PIVOT_HEIGHT) / dist);
        
        double clamped = MathUtil.clamp(this.shootingAngle, ShooterConstants.PIVOT_ANGLE_LIMITS[0], ShooterConstants.PIVOT_ANGLE_LIMITS[1]);
        if (this.shootingAngle != clamped) {
            ShooterSubsystem.getInstance().canShoot = false;
            end(true);
            return;
        }

        ShooterSubsystem.getInstance().pivotGoToPosition(this.shootingAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setPivotSpeed(0);
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(ShooterSubsystem.getInstance().getPivotPosition() - this.shootingAngle)
            <= ShooterConstants.ALLOWED_PIVOT_ERROR;
    }
}
