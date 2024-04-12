package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.CenterNoteCommand;
import frc.robot.auto.DriveToNoteCommand;
import frc.robot.constants.Constants.NoteConstants;
import frc.robot.constants.Constants.IntakeStates;
import frc.robot.constants.Constants.ShooterStates;
import frc.robot.intake.PivotIntakeCommand;
import frc.robot.intake.SpinIntakeCommand;
import frc.robot.shooter.PivotShooterMMCommand;
import frc.robot.shooter.ShootCommand;

/**
 * A class that stores command chains for use elsewhere.
 */
public class SequencedCommands {
    /**
     * Creates a command that moves the shooter and intake to intaking positions
     * and then runs the intaking command until it has a note in the sterilizer.
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
     * Creates a command that moves the shooter and intake to intaking positions,
     * runs the intaking command, center center the bot on the note and drives to it,
     * and upon intaking the note will return intake and shooter back to their idle positions.
     * @return the command
     */
    public static Command getCollectNoteCommand() {
        return Commands.sequence(
            Commands.parallel(
                new PivotIntakeCommand(IntakeStates.INTAKING),
                new PivotShooterMMCommand(ShooterStates.INTAKE)
            ),
            new CenterNoteCommand().withTimeout(NoteConstants.CENTERING_TIMEOUT),
            // Will end as soon as there is a note in the sterilizer.
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
     * Runs exactly like {@link SequencedCommands#getCollectNoteCommand()} but omits {@link CenterNoteCommand}.
     * @return the command
     */
    public static Command getCollectNoteCommandNoCenter() {
        return Commands.sequence(
            Commands.parallel(
                new PivotIntakeCommand(IntakeStates.INTAKING),
                new PivotShooterMMCommand(ShooterStates.INTAKE)
            ),
            // Will end as soon as there is a note in the sterilizer.
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
     * Runs exactly like {@link SequencedCommands#getCollectNoteCommandNoCenter()}
     * but does not return the shooter and intake to their idle positions.
     * @return the command
     */
    public static Command getCollectNoteCommandNoCenterNoIdle() {
        return Commands.sequence(
            Commands.parallel(
                new PivotIntakeCommand(IntakeStates.INTAKING),
                new PivotShooterMMCommand(ShooterStates.INTAKE)
            ),
            // Will end as soon as there is a note in the SpinIntakeCommand
            Commands.race(
                new SpinIntakeCommand(IntakeStates.INTAKING),
                new DriveToNoteCommand().withTimeout(3.5)
            )
        );
    }

    /**
     * Creates a command that pivots the shooter and shoots a note into the speaker from the current position.
     * @return the command
     */
    public static Command getAutoSpeakerShootCommand() {
        return Commands.sequence(
            new PivotShooterMMCommand(ShooterStates.SPEAKER_CALCULATE),
            new ShootCommand(ShooterStates.SPEAKER_CALCULATE)
        );
    }

    /**
     * Creates a command that shoots notes immediately after picking them up and never ends.
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