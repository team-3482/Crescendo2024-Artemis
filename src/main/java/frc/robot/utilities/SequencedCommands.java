package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.auto.CenterNoteCommand;
import frc.robot.auto.DriveToNoteCommand;
import frc.robot.intake.IntakePivotCommand;
import frc.robot.intake.SpinIntakeCommand;

public class SequencedCommands{
    public static Command intakeCommand(){
        return Commands.sequence(
            new IntakePivotCommand(IntakeState.INTAKING), 
            new SpinIntakeCommand(IntakeConstants.INTAKE_SPEED, 2.0)
        ).andThen(
            new IntakePivotCommand(IntakeState.IDLE)
        );
    }
    public static Command collectNote(){
        return Commands.sequence(
            new CenterNoteCommand(), 
            new IntakePivotCommand(IntakeState.INTAKING), 
            Commands.parallel(
                new SpinIntakeCommand(IntakeConstants.INTAKE_SPEED), 
                new DriveToNoteCommand()
            )
        ).andThen(
            new IntakePivotCommand(IntakeState.IDLE)
        );
    }
}