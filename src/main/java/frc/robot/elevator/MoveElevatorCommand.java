// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.ElevatorConstants;

/** A command that moves the shooter pivot to a desired position. */
public class MoveElevatorCommand extends Command {
    private final Supplier<Double> negativeSpeedSupplier;
    private final Supplier<Double> positiveSpeedSupplier;

    /**
    * Creates a new MoveElevatorCommand.
    * @param positiveSpeedSupplier speed from 0 to 1.0 to move the motors in the negative direction
    * @param negativeSpeedSupplier speed from 0 to 1.0 to move the motors in the positive direction 
    */
    public MoveElevatorCommand(Supplier<Double> positiveSpeedSupplier, Supplier<Double> negativeSpeedSupplier) {
        setName("MoveElevatorCommand");
        this.negativeSpeedSupplier = negativeSpeedSupplier;
        this.positiveSpeedSupplier = positiveSpeedSupplier;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ElevatorSubsystem.getInstance());
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double posSpeed = this.positiveSpeedSupplier.get();
            posSpeed = posSpeed < 0.05 ? 0 : posSpeed;
        double negSpeed = this.negativeSpeedSupplier.get();
            negSpeed = negSpeed < 0.05 ? 0 : negSpeed;
        double speed;
        
        if (posSpeed != 0 && negSpeed != 0) {
            speed = 0;
        }
        else {
            speed = posSpeed - negSpeed;
        }
        
        if (Math.abs(speed) >= 0.5) {
            speed = Math.signum(speed) * ElevatorConstants.MOVING_SPEED;
        }
        else {
            speed = 0;
        }

        ElevatorSubsystem.getInstance().setSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ElevatorSubsystem.getInstance().setSpeed();
    }

    // Returns true when the command should end. It never ends due to being a default command.
    @Override
    public boolean isFinished() {
        return false;
    }
}
