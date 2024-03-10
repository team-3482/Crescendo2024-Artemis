package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

// Copied from Commands.runOnce()
public class RunOnceDisabled extends InstantCommand {
    /**
     * Constructs a command that runs an action once and finishes.
     * It can run while the bot is disabled.
     *
     * @param action the action to run
     * @param requirements subsystems the action requires
     * @see InstantCommand
     */
    public RunOnceDisabled(Runnable action, Subsystem... requirements) {
        super(action, requirements);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
