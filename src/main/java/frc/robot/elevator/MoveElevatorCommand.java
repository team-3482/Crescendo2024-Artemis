// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command that moves the shooter pivots.
 */
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

        addRequirements(ElevatorSubsystem.getInstance());
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double posSpeed = this.positiveSpeedSupplier.get();
            posSpeed = posSpeed < 0.05 ? 0 : posSpeed; // Deadband for trigger
        double negSpeed = this.negativeSpeedSupplier.get();
            negSpeed = negSpeed < 0.05 ? 0 : negSpeed; // Deadband for trigger
        double speed;
        
        if (posSpeed != 0 && negSpeed != 0) {
            speed = 0;
        }
        else {
            speed = (posSpeed - negSpeed) * 0.25;
            speed = speed <= -0.20 ? -0.5 : speed;
        }

        ElevatorSubsystem.getInstance().setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        ElevatorSubsystem.getInstance().setSpeed();
    }

    // This command will always never end due to being a default command.
    @Override
    public boolean isFinished() {
        return false;
    }
}
