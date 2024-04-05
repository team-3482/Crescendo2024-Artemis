// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that moves the shooter pivot to a desired position. */
public class ManuallyPivotShooterCommand extends Command {
    private final Supplier<Double> leftSpeedSupplier;
    private final Supplier<Double> rightSpeedSupplier;
    private final boolean swapSides;

    /**
    * Creates a new ManuallyPivotShooterCommand.
    * @param leftSpeedSupplier speed from -1.0 to 1.0 to move the left motor
    * @param rightSpeedSupplier speed from -1.0 to 1.0 to move the right motor
    * @param swapSides swap suppliers and motors
    */
    public ManuallyPivotShooterCommand(Supplier<Double> leftSpeedSupplier, Supplier<Double> rightSpeedSupplier, boolean swapSides) {
        setName("ManuallyPivotShooterCommand");
        this.leftSpeedSupplier = leftSpeedSupplier;
        this.rightSpeedSupplier = rightSpeedSupplier;
        this.swapSides = swapSides;
        // Default commands need to require their subsystem.
        addRequirements(ShooterSubsystem.getInstance());
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double leftSpeed = leftSpeedSupplier.get();
        double rightSpeed = rightSpeedSupplier.get();
        
        if (Math.abs(leftSpeed) >= 0.85) {
            leftSpeed = Math.signum(leftSpeed) * 0.2;
        }
        else if (Math.abs(leftSpeed) >= 0.15) {
            leftSpeed = Math.signum(leftSpeed) * 0.1;
        }
        else {
            leftSpeed = 0;
        }

        if (Math.abs(rightSpeed) >= 0.85) {
            rightSpeed = Math.signum(rightSpeed) * 0.2;
        }
        else if (Math.abs(rightSpeed) >= 0.15) {
            rightSpeed = Math.signum(rightSpeed) * 0.1;
        }
        else {
            rightSpeed = 0;
        }

        ShooterSubsystem.getInstance().setPivotSpeed(
            swapSides ? rightSpeed : leftSpeed,
            swapSides ? leftSpeed : rightSpeed,
            true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setPivotSpeed(0, false);
    }

    // Returns true when the command should end. It never ends due to being a default command.
    @Override
    public boolean isFinished() {
        return false;
    }
}
