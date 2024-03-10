package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.auto.DriveToNoteCommand;
import frc.robot.intake.CenterNoteCommand;
import frc.robot.intake.PivotIntakeCommand;
import frc.robot.intake.SpinIntakeCommand;
import frc.robot.shooter.PivotShooterCommand;

public class SequencedCommands{

    /**
     * Creates a command that drops the intake and turns on the motor speed
     * @return a command to set the intake to the correct position and turns the motors on
     */
    public static Command intakeCommand(){
        return Commands.sequence(
            Commands.parallel(
            new PivotIntakeCommand(IntakeState.INTAKING), new PivotShooterCommand(ShooterState.INTAKE)), 
            new SpinIntakeCommand(IntakeConstants.INTAKE_SPEED, 2.0)
            
        ).andThen(
            new PivotIntakeCommand(IntakeState.IDLE)
        );
    }
    /**
     * Creates a command that automatically aligns to a note and picks up the note
     * @return a command to auto align and pick up a note 
     */
    public static Command collectNote(){
        return Commands.sequence(
            new CenterNoteCommand(), 
            new PivotIntakeCommand(IntakeState.INTAKING), 
            Commands.parallel(
                new SpinIntakeCommand(IntakeConstants.INTAKE_SPEED), 
                new DriveToNoteCommand()
            )
        ).andThen(
            new PivotIntakeCommand(IntakeState.IDLE)
        );
    }
}