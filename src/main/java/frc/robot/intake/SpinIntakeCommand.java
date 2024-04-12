// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.IntakeStates;
import frc.robot.constants.PhysicalConstants.SterilizerConstants;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.shooter.SterilizerSubsystem;
import frc.robot.utilities.Telemetry;

/**
 * A command to spin the rollers in the intake.
 * */
public class SpinIntakeCommand extends Command {
    private IntakeStates state;
    private boolean stopForNote;
    private boolean finished;

    /**
     * Creates a new SpinIntakeCommand.
     * @param state state of the intake.
     * @param stopForNote stop the command when a note is in the sterilizer.
     * @apiNote if {@code stopForNote} is {@code false}, this subsystem will NOT run the sterilizer.
     * Use it in conjunction with {@link ShootCommmand}.
     */
    public SpinIntakeCommand(IntakeStates state, boolean stopForNote) {
        setName("IntakeCommand");

        this.state = state;
        this.stopForNote = stopForNote;

        // Dont't require the Sterilizer when not stopping for notes,
        // otherwise it conflicts with parallel commands that require it.
        if (stopForNote) {
            addRequirements(IntakeSubsystem.getInstance().getIntakingRequirement(), SterilizerSubsystem.getInstance());
        }
        else {
            addRequirements(IntakeSubsystem.getInstance().getIntakingRequirement());
        }
    }

    /**
     * Creates a new SpinIntakeCommand that stops for notes.
     * @param state state of the intake.
     */
    public SpinIntakeCommand(IntakeStates state) {
        this(state, true);
    }

    @Override
    public void initialize() {
        this.finished = false;

        LEDSubsystem.getInstance().setLightState(LightState.CMD_RUNNING);
    }

    @Override
    public void execute() {
        IntakeSubsystem.getInstance().setIntakeSpeed(this.state.getSpeed());
        
        if (!this.stopForNote) return;

        Optional<Boolean>[] hasNotesOptionals = SterilizerSubsystem.getInstance().getHasNotes();
        boolean[] hasNotes = new boolean[]{
            hasNotesOptionals[0].isPresent() && hasNotesOptionals[0].get(),
            hasNotesOptionals[1].isPresent() && hasNotesOptionals[1].get()
        };

        if (this.state.getSpeed() < 0) {
            SterilizerSubsystem.getInstance().setSpeed(-SterilizerConstants.FEEDING_SPEED);
        }
        else if (!hasNotes[0] && !hasNotes[1]) {
            SterilizerSubsystem.getInstance().setSpeed(SterilizerConstants.FEEDING_SPEED);
        }
        else if (hasNotes[0] && !hasNotes[1]) {
            SterilizerSubsystem.getInstance().setSpeed(SterilizerConstants.ADJUSTING_SPEED);
        }
        else if (hasNotes[1]) {
            this.finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {IntakeSubsystem.getInstance().setIntakeSpeed();
        SterilizerSubsystem.getInstance().setSpeed();

        Telemetry.logCommandEnd(getName(), interrupted);
        LEDSubsystem.getInstance().setCommandStopState(interrupted);
    }

    @Override
    public boolean isFinished() {
        return this.finished;
    }
}
