// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;

public class Intake extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    public Intake() {
        this.addRequirements(IntakeSubsystem.getInstance());  
    }

    @Override
    public void execute() {
        IntakeSubsystem.getInstance().setIntakeMotor(IntakeConstants.INTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setIntakeMotor(0);
    }
}
