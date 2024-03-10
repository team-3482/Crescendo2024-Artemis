package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.intake.CenterNoteCommand;
import frc.robot.intake.DriveToNoteCommand;
import frc.robot.intake.PivotIntakeCommand;
import frc.robot.intake.SpinIntakeCommand;
import frc.robot.shooter.PivotShooterCommand;

/** A class that stores command chains for use elsewhere */
public class SequencedCommands{
    /**
     * Creates a command that moves the shooter and intake to intaking positions and then turns on the motors
     * infinitely or until it has a note in the sterilizer.
     * @return the command
     */
    public static Command getIntakeCommand() {
        return Commands.sequence(
            Commands.parallel(
                new PivotShooterCommand(ShooterState.INTAKE),
                new PivotIntakeCommand(IntakeState.INTAKING)
            ),
            new SpinIntakeCommand(IntakeConstants.INTAKE_SPEED)
        );
    }

    /**
     * Creates a command that moves the shooter and intake to intaking positions and then turns on the motors
     * until it has a note in the sterilizer or the limelight does not see the note anymore.
     * @return the command
     */
    public static Command getCollectNoteCommand() {
        return Commands.sequence(
            new CenterNoteCommand(),
            Commands.parallel(
                new PivotShooterCommand(ShooterState.INTAKE),
                new PivotIntakeCommand(IntakeState.INTAKING)
            ),
            // Will end early if DriveToNoteCommand() no longer sees the note
            // If there is a note in the sterilizer, both would end anyways
            Commands.race(
                new SpinIntakeCommand(IntakeConstants.INTAKE_SPEED), 
                new DriveToNoteCommand()
            ),
            new PivotIntakeCommand(IntakeState.IDLE)
        );
    }
}