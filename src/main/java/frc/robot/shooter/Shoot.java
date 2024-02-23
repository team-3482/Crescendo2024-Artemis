// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;

public class Shoot extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final ShootSubsystem m_subsystem;

    public Shoot(ShootSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        m_subsystem.SetShooterMotors(ShooterConstants.SHOOTER_SPEED, ShooterConstants.FEEDER_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.SetShooterMotors(0, 0);
    }
}
