// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OrbitConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.swerve.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class ShootCommand extends Command {
    private double[] shootingSpeed;
    private double shootingAngle;
    private boolean finished;

    /**
    * Creates a new ShootCommand.
    *
    * @param subsystem The subsystem used by this command.
    */
    public ShootCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem.getInstance());
    }

    /**
     * Calculate the shooting speeds for the motors 
     * 
     * @return velocities for left [0] and right [1] motors
     */
    private double[] getShootingSpeed() {
        double[] _speeds = ShooterConstants.SHOOTER_MOTOR_SPEEDS;
        if (SwerveSubsystem.getInstance().getHeading() < 180) {
            return _speeds; 
        }
        return new double[]{_speeds[1], _speeds[0]}; 
    }

    /**
     * Calculate the shooting angle
     * 
     * @return angle for the pivot
     */
    private double getShootingAngle() {
        Translation3d point = OrbitConstants.ORBIT_POINT.get(DriverStation.getAlliance().get());
        Pose2d botpose = SwerveSubsystem.getInstance().getPose();
        
        double dist = Math.sqrt(
            Math.pow(point.getX() - botpose.getX(), 2) + 
            Math.pow(point.getY() - botpose.getY(), 2)
        );
        double angle = Math.atan((point.getZ() - PhysicalConstants.SHOOTER_PIVOT_HEIGHT) / dist);
        return angle; 
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        LEDSubsystem.getInstance().setLightState(LightState.CMD_INIT);
        // Double so it can be null if the ID cannot be orbited
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            // Will set warning light
            end(true);
            return;
        }

        this.finished = false;
        this.shootingSpeed = getShootingSpeed();
        this.shootingAngle = getShootingAngle();
        ShooterSubsystem.getInstance().setPivotPosition(shootingAngle);
        ShooterSubsystem.getInstance().setShootingVelocities(new double[]{
            shootingSpeed[0], shootingSpeed[1]
        });
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);

        double position = ShooterSubsystem.getInstance().getPivotPosition();
        if (Math.abs(this.shootingAngle - position) > ShooterConstants.ALLOWED_PIVOT_ERROR) return;
        
        double[] velocities = ShooterSubsystem.getInstance().getShootingVelocities();
        if (Units.radiansPerSecondToRotationsPerMinute(this.shootingSpeed[0] - velocities[0]) > ShooterConstants.ALLOWED_RPM_ERROR
            || Units.radiansPerSecondToRotationsPerMinute(this.shootingSpeed[1] - velocities[1]) > ShooterConstants.ALLOWED_RPM_ERROR
        ) return;

        Optional<Boolean> hasNote = SterilizerSubsystem.getInstance().hasNote(); 
        if (!hasNote.isPresent() && hasNote.get()) {
            SterilizerSubsystem.getInstance().moveForward();
        }
        Timer.delay(1);
        this.finished = true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setShootingVelocities(new double[]{0, 0});
        ShooterSubsystem.getInstance().setPivotPosition(0);
        SterilizerSubsystem.getInstance().moveStop();

        if (interrupted) {
            LEDSubsystem.getInstance().setLightState(LightState.WARNING);
        }
        else {
            LEDSubsystem.getInstance().setLightState(LightState.OFF);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.finished;
    }
}
