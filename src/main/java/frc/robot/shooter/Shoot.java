// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;

public class Shoot extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    public Shoot() {
        this.addRequirements(ShootSubsystem.getInstance());
    }

    @Override
    public void execute() {
        ShootSubsystem.getInstance().SetShooterMotors(ShooterConstants.SHOOTER_SPEED, ShooterConstants.FEEDER_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        ShootSubsystem.getInstance().SetShooterMotors(0, 0);
    }
}
