package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.NoteConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.intake.CenterNoteCommand;
import frc.robot.intake.DriveToNoteCommand;
import frc.robot.intake.PivotIntakeCommand;
import frc.robot.intake.SpinIntakeCommand;
import frc.robot.shooter.PivotShooterCommand;
import frc.robot.shooter.RevUpCommand;
import frc.robot.swerve.CenterSpeakerCommand;

/** A class that stores command chains for use elsewhere */
public class SequencedCommands {
    /**
     * Creates a command that moves the shooter and intake to intaking positions and then turns on the motors
     * infinitely or until it has a note in the sterilizer.
     * @return the command
     */
    public static Command getIntakeCommand() {
        return Commands.parallel(
            new PivotShooterCommand(ShooterState.INTAKE),
            new PivotIntakeCommand(IntakeState.INTAKING),
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
            Commands.parallel(
                new PivotIntakeCommand(IntakeState.INTAKING),
                new PivotShooterCommand(ShooterState.INTAKE)
            ),
            new CenterNoteCommand().withTimeout(NoteConstants.CENTERING_TIMEOUT),
            // Will end as soon as there is a note in the SpinIntakeCommand
            Commands.race(
                new SpinIntakeCommand(IntakeConstants.INTAKE_SPEED), 
                new DriveToNoteCommand().withTimeout(5)
            ),
            Commands.parallel(
                new PivotIntakeCommand(IntakeState.IDLE),
                new PivotShooterCommand(ShooterState.SPEAKER)
            )
        );
    }
    
    /**
     * Creates a command that moves the shooter and intake to intaking positions and then turns on the motors
     * until it has a note in the sterilizer or the limelight does not see the note anymore.
     * @return the command
     */
    public static Command getCollectNoteCommandNoCenter() {
        return Commands.sequence(
            Commands.parallel(
                new PivotIntakeCommand(IntakeState.INTAKING),
                new PivotShooterCommand(ShooterState.INTAKE),
                new SpinIntakeCommand(IntakeConstants.INTAKE_SPEED)
            ),
            // Will end as soon as there is a note in the SpinIntakeCommand
            // Commands.race(
            new DriveToNoteCommand().withTimeout(3),
            // ),
            Commands.parallel(
                new PivotIntakeCommand(IntakeState.IDLE),
                new PivotShooterCommand(ShooterState.SPEAKER)
            )
        );
    }

    /**
     * Creates a command that shoots a note into the speaker automatically from the current position.
     * 
     * @return the command
     */
    public static Command getAutoSpeakerShootCommand() {
        return Commands.sequence(
            new CenterSpeakerCommand(),
            new PivotShooterCommand(ShooterState.SPEAKER_CALCULATE),
            new RevUpCommand(ShooterState.SPEAKER_CALCULATE)
        );
    }
}