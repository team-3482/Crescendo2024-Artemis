package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.CenterNoteCommand;
import frc.robot.auto.CenterSpeakerCommand;
import frc.robot.auto.DriveToNoteCommand;
import frc.robot.constants.Constants.NoteConstants;
import frc.robot.constants.Constants.IntakeStates;
import frc.robot.constants.Constants.ShooterStates;
import frc.robot.intake.PivotIntakeCommand;
import frc.robot.intake.SpinIntakeCommand;
import frc.robot.shooter.PivotShooterMMCommand;
import frc.robot.shooter.RevUpCommand;
import frc.robot.shooter.ShootCommand;

/** A class that stores command chains for use elsewhere */
public class SequencedCommands {
    /**
     * Creates a command that moves the shooter and intake to intaking positions and then turns on the motors
     * infinitely or until it has a note in the sterilizer.
     * @return the command
     */
    public static Command getIntakeCommand() {
        return Commands.parallel(
            new PivotShooterMMCommand(ShooterStates.INTAKE),
            new PivotIntakeCommand(IntakeStates.INTAKING),
            new SpinIntakeCommand(IntakeStates.INTAKING)
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
                new PivotIntakeCommand(IntakeStates.INTAKING),
                new PivotShooterMMCommand(ShooterStates.INTAKE)
            ),
            new CenterNoteCommand().withTimeout(NoteConstants.CENTERING_TIMEOUT),
            // Will end as soon as there is a note in the SpinIntakeCommand
            Commands.race(
                new SpinIntakeCommand(IntakeStates.INTAKING),
                new DriveToNoteCommand().withTimeout(3.5)
            ),
            Commands.parallel(
                new PivotIntakeCommand(IntakeStates.IDLE),
                new PivotShooterMMCommand(ShooterStates.SPEAKER)
            )
        );
    }
    
    /**
     * Creates a command that moves the shooter and intake to intaking positions and then turns on the motors
     * until it has a note in the sterilizer
     * @return the command
     */
    public static Command getCollectNoteCommandNoCenter() {
        return Commands.sequence(
            Commands.parallel(
                new PivotIntakeCommand(IntakeStates.INTAKING),
                new PivotShooterMMCommand(ShooterStates.INTAKE)
            ),
            // Will end as soon as there is a note in the SpinIntakeCommand
            Commands.race(
                new SpinIntakeCommand(IntakeStates.INTAKING),
                new DriveToNoteCommand().withTimeout(3)
            ),
            Commands.parallel(
                new PivotIntakeCommand(IntakeStates.IDLE),
                new PivotShooterMMCommand(ShooterStates.SPEAKER)
            )
        );
    }
    /**
     * Creates a command that moves the shooter and intake to intaking positions and then turns on the motors
     * until it has a note in the sterilizer. It does not return the pivots to the idle positions because 
     * other commands will move them if they need the pivots to be moved (decreses time required for command)
     * @return the command
     */
    public static Command getAutonCollectNoteCommand() {
        return Commands.sequence(
            Commands.parallel(
                new PivotIntakeCommand(IntakeStates.INTAKING),
                new PivotShooterMMCommand(ShooterStates.INTAKE)
            ),
            // Will end as soon as there is a note in the SpinIntakeCommand
            Commands.race(
                new SpinIntakeCommand(IntakeStates.INTAKING),
                new DriveToNoteCommand()
            )
        );
    }

    /**
     * Creates a command that shoots a note into the speaker automatically from the current position.
     * @return the command
     */
    public static Command getAutoSpeakerShootCommand() {
        return Commands.sequence(
            new RevUpCommand(ShooterStates.SPEAKER_CALCULATE),
            new CenterSpeakerCommand().withTimeout(1.5),
            new PivotShooterMMCommand(ShooterStates.SPEAKER_CALCULATE),
            new ShootCommand(ShooterStates.SPEAKER_CALCULATE)
        );
    }

    /**
     * Creates a command that shoots notes immediately after picking them up and doesn't close the intake
     * @return the command
     */
    public static Command getIntakeEjectCommand() {
        return Commands.parallel(
            new PivotIntakeCommand(IntakeStates.INTAKING),
            new PivotShooterMMCommand(ShooterStates.FRONT_EJECT),

            new SpinIntakeCommand(IntakeStates.INTAKING, false),
            new ShootCommand(ShooterStates.FRONT_EJECT)
        );
    }
}