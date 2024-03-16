// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;

/** A command that moves the shooter pivot to a desired position. */
public class PivotShooterCommand extends Command {
    private double shootingAngle;
    private ShooterState state;

    /**
    * Creates a new PivotShooterCommand.
    * 
    * @param state of the shooter to reach
    */
    public PivotShooterCommand(ShooterState state) {
        setName("PivotShooterCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem.getInstance());
        this.state = state;
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        LEDSubsystem.getInstance().setLightState(LightState.CMD_INIT);
        ShooterSubsystem.getInstance().canShoot = true;
        if(!this.state.getCalculateAngle()) {
            this.shootingAngle = this.state.getAngle();
            ShooterSubsystem.getInstance().pivotGoToPosition(this.shootingAngle);
            LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);
            return;
        }
        
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            end(true);
            return;
        }

        // TODO Calculating shooter angles

        // Translation3d point = OrbitConstants.ORBIT_POINT.get(DriverStation.getAlliance().get());
        // Pose2d botpose = SwerveSubsystem.getInstance().getPose();
        
        // double dist = Math.sqrt(
        //     Math.pow(point.getX() - botpose.getX(), 2) + 
        //     Math.pow(point.getY() - botpose.getY(), 2)
        // );
        // this.shootingAngle = Math.atan((point.getZ() - PhysicalConstants.SHOOTER_PIVOT_HEIGHT) / dist);
        
        // double clamped = MathUtil.clamp(this.shootingAngle, ShooterConstants.PIVOT_ANGLE_LIMITS[0], ShooterConstants.PIVOT_ANGLE_LIMITS[1]);
        // if (this.shootingAngle != clamped) {
        //     end(true);
        //     return;
        // }

        // ShooterSubsystem.getInstance().pivotGoToPosition(this.shootingAngle);

        LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            ShooterSubsystem.getInstance().canShoot = false;
        }
        ShooterSubsystem.getInstance().setPivotSpeed(0, false);
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(ShooterSubsystem.getInstance().getPivotPositions()[1] - this.shootingAngle)
            <= ShooterConstants.ALLOWED_PIVOT_ERROR;
    }
}
