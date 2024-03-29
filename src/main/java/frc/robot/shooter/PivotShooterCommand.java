// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.ShooterConstants;
import frc.robot.constants.Constants.ShooterStates;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.utilities.Telemetry;

/** A command that moves the shooter pivot to a desired position. */
public class PivotShooterCommand extends Command {
    private double shootingAngle;
    private ShooterStates state;
    private ProfiledPIDController pid;

    /**
    * Creates a new PivotShooterCommand.
    * @param state of the shooter to reach
    */
    public PivotShooterCommand(ShooterStates state) {
        setName("PivotShooterCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        this.state = state;
        this.pid = new ProfiledPIDController(ShooterConstants.kP_PIVOT, 0, 0,
            new TrapezoidProfile.Constraints(30, 30)
        );

        addRequirements(ShooterSubsystem.getInstance());
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double[] pivotPositions = ShooterSubsystem.getInstance().getPivotPositions();
        this.pid.reset(pivotPositions[0]);

        if(!this.state.getCalculateAngle()) {
            this.shootingAngle = this.state.getAngle();
            LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);
            return;
        }
        
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            end(true);
            return;
        }

        // // Calculating shooter angles based off current bot position
        // Translation3d point = ShooterConstants.SPEAKER_LOCATIONS.get(DriverStation.getAlliance().get());
        // Pose2d botpose = SwerveSubsystem.getInstance().getPose();

        // // Calculates horizontal distance to speaker
        // double dist = Math.sqrt(
        //     Math.pow(point.getX() - botpose.getX(), 2) + 
        //     Math.pow(point.getY() - botpose.getY(), 2)
        // );
        // // Calculates angle from bot distance to speaker height
        // this.shootingAngle = Units.radiansToDegrees(Math.atan((point.getZ() - PhysicalConstants.SHOOTER_PIVOT_HEIGHT) / dist));
        
        // // Telemetry.logMessage("Bot Position: (" + botpose.getX() + "," + botpose.getY()+")");
        // // Telemetry.logMessage("Speaker Position: (" + point.getX() + "," + point.getY()+"," + point.getZ()+")");
        // // Telemetry.logMessage("Distance: " + dist);
        // // Telemetry.logMessage("Shooting Angle " + this.shootingAngle);
        
        // // Checks if the angle is within the current bounds, if not, ends the command 
        // double clamped = MathUtil.clamp(this.shootingAngle, ShooterConstants.PIVOT_ANGLE_LIMITS[0], ShooterConstants.PIVOT_ANGLE_LIMITS[1]);
        // if (this.shootingAngle != clamped) {
        //     end(true);
        //     return;
        // }

        // ShooterSubsystem.getInstance().pivotGoToPosition(this.shootingAngle);
        
        // LEDSubsystem.getInstance().setLightState(LightState.AUTO_RUNNING);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double[] pivotPositions = ShooterSubsystem.getInstance().getPivotPositions();
        ShooterSubsystem.getInstance().setPivotSpeed(
            this.pid.calculate(pivotPositions[0], this.shootingAngle),
            this.pid.calculate(pivotPositions[1], this.shootingAngle),
            true
        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setPivotSpeed(0, false);
        ShooterSubsystem.getInstance().canShoot = !interrupted;

        Telemetry.logCommandEnd(getName(), interrupted, "goal " + Telemetry.D_FORMAT.format(this.shootingAngle));
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double[] pivotPositions = ShooterSubsystem.getInstance().getPivotPositions();
        return Math.abs(pivotPositions[0] - this.shootingAngle) <= ShooterConstants.ALLOWED_PIVOT_ERROR
            && Math.abs(pivotPositions[1] - this.shootingAngle) <= ShooterConstants.ALLOWED_PIVOT_ERROR;
    }
}
