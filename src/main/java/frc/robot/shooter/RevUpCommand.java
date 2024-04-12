// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ShooterStates;
import frc.robot.utilities.Telemetry;

/**
 * A command that spins the shooting wheels to a desired speed and never ends.
 */
public class RevUpCommand extends Command {
    private double[] rpms;
    private ShooterStates state;

    /**
     * Creates a new RevUpCommand.
     * Revs up both motors to the provided RPM.
     * @param velocity in RPM.
     */
    public RevUpCommand(double velocity) {
        setName("RevUpCommand");

        this.rpms = new double[]{velocity, velocity};
        this.state = null;
        
        addRequirements(ShooterSubsystem.getInstance().getShootingRequirement());
    }

    /**
    * Creates a new RevUpCommand.
    * Revs up to 80% of lower RPM (or high if none) on both sides.
    * @param state the shooter state to get RPMs from.
    */
    public RevUpCommand(ShooterStates state) {
        setName("RevUpCommand");
        
        this.state = state;
        
        addRequirements(ShooterSubsystem.getInstance().getShootingRequirement());
    }
    
    @Override
    public void initialize() {
        if (this.state != null) {
            double rpm = this.state.getRPMs(false)[1];
            this.rpms = new double[]{rpm, rpm};
        }

        ShooterSubsystem.getInstance().setShootingVelocities(this.rpms);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        // This command should be getting interrupted most of the time.
        Telemetry.logCommandEnd(getName(), interrupted);
    }

    /**
     * This command never ends, it must be interrupted.
     * @return {@code false}
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
