// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.PhysicalConstants.RobotConstants;
import frc.robot.constants.PhysicalConstants.ShooterConstants;
import frc.robot.constants.Positions;
import frc.robot.constants.Constants.ShooterStates;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utilities.Telemetry;

/** A command that moves the shooter pivot to a desired position. */
public class PivotShooterMMCommand extends Command {
    private double shootingAngle;
    private ShooterStates state;

    /**
    * Creates a new PivotShooterCommand.
    * @param state of the shooter to reach
    */
    public PivotShooterMMCommand(ShooterStates state) {
        setName("PivotShooterCommand");
        this.state = state;
        
        // Use addRequirements() here  to declare subsystem dependencies.
        addRequirements(ShooterSubsystem.getInstance().getPivotRequirement());
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ShooterSubsystem.getInstance().setRotorPositions();

        if(!this.state.getCalculateAngle()) {
            this.shootingAngle = this.state.getAngle();
            LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);
            ShooterSubsystem.getInstance().pivotGoToPosition(this.shootingAngle);
            return;
        }

        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            CommandScheduler.getInstance().cancel(this);
            return;
        }

        // Calculating shooter angles based off current bot position
        Translation3d point = Positions.SPEAKER_TARGETS.get(DriverStation.getAlliance().get());
        Pose2d botpose = SwerveSubsystem.getInstance().getPose();

        // Calculates horizontal distance to speaker
        double dist = Math.sqrt(
            Math.pow(point.getX() - botpose.getX(), 2) +
            Math.pow(point.getY() - botpose.getY(), 2)
        );
        
        // Calculates angle from bot distance to speaker height
        this.shootingAngle = Units.radiansToDegrees(Math.atan((point.getZ() - RobotConstants.SHOOTER_PIVOT_HEIGHT) / dist));
        
        // Checks if the angle is within the current bounds, if not, ends the command 
        this.shootingAngle = MathUtil.clamp(this.shootingAngle, ShooterConstants.Pivot.ANGLE_LIMITS[0], ShooterConstants.Pivot.ANGLE_LIMITS[1]);
        
        ShooterSubsystem.getInstance().pivotGoToPosition(this.shootingAngle);

        LEDSubsystem.getInstance().setLightState(LightState.AUTO_RUNNING);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setPivotSpeed(0, false);

        Telemetry.logCommandEnd(getName(), interrupted, "goal " + Telemetry.D_FORMAT.format(this.shootingAngle));
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double[] pivotPositions = ShooterSubsystem.getInstance().getCANcoderPositions();
        return Math.abs(pivotPositions[0] - this.shootingAngle) <= ShooterConstants.Pivot.ALLOWED_ERROR
            && Math.abs(pivotPositions[1] - this.shootingAngle) <= ShooterConstants.Pivot.ALLOWED_ERROR;
    }
}
