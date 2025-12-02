package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.wheel.Wheel;

/**
 * Triggers the single shot sequence.
 * The actual logic is now handled by the Wheel subsystem's state machine.
 */
public class LaunchSingleCommand extends InstantCommand {

    public LaunchSingleCommand(Wheel wheel) {
        super(
            wheel::fire,
            wheel
        );
    }
}
