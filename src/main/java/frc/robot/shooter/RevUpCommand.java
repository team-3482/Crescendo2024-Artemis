// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ShooterStates;
import frc.robot.utilities.Telemetry;

/** A command that spins the large wheels of the shooter at the desired speed. */
public class RevUpCommand extends Command {
    private double[] rpms;
    private ShooterStates state;

    /**
     * Creates a new RevUpCommand.
     * Revs up both motors to the provided RPM
     * @param velocity in RPMs
     */
    public RevUpCommand(double velocity) {
        setName("RevUpCommand");
        this.rpms = new double[]{velocity, velocity};
        this.state = null;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem.getInstance().getShootingRequirement());
    }

    /**
    * Creates a new RevUpCommand.
    * Revs up to 80% of lower RPM (or high if none) on both sides
    *
    * @param state the shooter state to get RPMs from
    */
    public RevUpCommand(ShooterStates state) {
        setName("RevUpCommand");
        this.state = state;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem.getInstance().getShootingRequirement());
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (this.state != null) {
            double[] _rpms = this.state.getRPMs(false);
            this.rpms = new double[]{_rpms[1], _rpms[1]};
        }
        ShooterSubsystem.getInstance().setShootingVelocities(this.rpms);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // This command should be getting interrupted most of the time
        Telemetry.logCommandEnd(getName(), interrupted);
    }

    /**
     * This command never ends, it must be interrupted.
     * @return false
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
