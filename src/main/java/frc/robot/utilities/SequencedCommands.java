package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.CenterNoteCommand;
import frc.robot.auto.CenterSpeakerCommand;
import frc.robot.auto.DriveToNoteCommand;
import frc.robot.constants.Constants.NoteConstants;
import frc.robot.constants.Constants.IntakeConstants.IntakeState;
import frc.robot.constants.Constants.ShooterConstants.ShooterState;
import frc.robot.intake.PivotIntakeCommand;
import frc.robot.intake.SpinIntakeCommand;
import frc.robot.shooter.PivotShooterCommand;
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
            new PivotShooterCommand(ShooterState.INTAKE),
            new PivotIntakeCommand(IntakeState.INTAKING),
            new SpinIntakeCommand(IntakeState.INTAKING)
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
                new SpinIntakeCommand(IntakeState.INTAKING), 
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
     * until it has a note in the sterilizer
     * @return the command
     */
    public static Command getCollectNoteCommandNoCenter() {
        return Commands.sequence(
            Commands.parallel(
                new PivotIntakeCommand(IntakeState.INTAKING),
                new PivotShooterCommand(ShooterState.INTAKE)
            ),
            // Will end as soon as there is a note in the SpinIntakeCommand
            Commands.deadline(
                new SpinIntakeCommand(IntakeState.INTAKING),
                new DriveToNoteCommand().withTimeout(4)
            ),
            Commands.parallel(
                new PivotIntakeCommand(IntakeState.IDLE),
                new PivotShooterCommand(ShooterState.SPEAKER)
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
                new PivotIntakeCommand(IntakeState.INTAKING),
                new PivotShooterCommand(ShooterState.INTAKE)
            ),
            // Will end as soon as there is a note in the SpinIntakeCommand
            Commands.deadline(
                new SpinIntakeCommand(IntakeState.INTAKING),
                new DriveToNoteCommand().withTimeout(4)
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
            Commands.parallel(
                new CenterSpeakerCommand(),
                new PivotShooterCommand(ShooterState.SPEAKER_CALCULATE)
            ),
            new ShootCommand(ShooterState.SPEAKER_CALCULATE)
        );
    }

    /**
     * Creates a command that shoots notes immediately after picking them up and doesn't close the intake
     * 
     * @return the command
     */
    public static Command getIntakeEjectCommand() {
        return Commands.sequence(
            Commands.parallel(
                new PivotIntakeCommand(IntakeState.INTAKING),
                new PivotShooterCommand(ShooterState.FRONT_EJECT)
            ),    
            Commands.parallel(
                new SpinIntakeCommand(IntakeState.INTAKING, false),
                new ShootCommand(ShooterState.FRONT_EJECT)
            )
        );
    }
}