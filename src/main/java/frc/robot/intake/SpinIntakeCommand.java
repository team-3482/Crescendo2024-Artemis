// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SterilizerConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.shooter.SterilizerSubsystem;
import frc.robot.utilities.Telemetry;

/** A command to spin the rollers in the intake using the current {@link IntakeState}. */
public class SpinIntakeCommand extends Command {
    private IntakeState state;
    private boolean stopForNote;

    /**
     * Initializes a new SpinIntakeCommand
     * 
     * @param state state of the intake
     * @param stopForNote stop the command when a note is in the sterilizer
     */
    public SpinIntakeCommand(IntakeState state, boolean stopForNote) {
        setName("IntakeCommand");
        this.state = state;
        this.stopForNote = stopForNote;
    }

    /**
     * Initializes a new SpinIntakeCommand that stops for notes (overloaded)
     * 
     * @param state state of the intake
     */
    public SpinIntakeCommand(IntakeState state) {
        this(state, true);
    }

    @Override
    public void initialize() {
        LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);
    }

    @Override
    public void execute() {
        IntakeSubsystem.getInstance().setIntakeSpeed(this.state.getSpeed());
        Optional<Boolean>[] hasNotesOptionals = SterilizerSubsystem.getInstance().getHasNotes();
        boolean[] hasNotes = new boolean[]{
            hasNotesOptionals[0].isPresent() && hasNotesOptionals[0].get(),
            hasNotesOptionals[1].isPresent() && hasNotesOptionals[1].get()
        };

        if (this.state.getSpeed() < 0) {
            SterilizerSubsystem.getInstance().setSpeed(-SterilizerConstants.FEEDING_SPEED);
        }
        else if (!this.stopForNote || (!hasNotes[0] && !hasNotes[1])) {
            SterilizerSubsystem.getInstance().setSpeed(SterilizerConstants.FEEDING_SPEED);
        }
        else if (hasNotes[0] && !hasNotes[1]) {
            SterilizerSubsystem.getInstance().setSpeed(SterilizerConstants.ADJUSTING_SPEED);
        }
        else if (hasNotes[0] && hasNotes[1]) {
            end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setIntakeSpeed(0);
        SterilizerSubsystem.getInstance().setSpeed();

        LEDSubsystem.getInstance().setCommandStopState(interrupted);
        Telemetry.logMessage(this.getName(), interrupted);
    }

    /**
     * Will return false because the {@link SpinIntakeCommand#execute()} loop will end this command
     * 
     * @return false
     */
    @Override
    public boolean isFinished() {
        // return this.stopForNote && SterilizerSubsystem.getInstance().hasNote();
        return false;
    }
}
