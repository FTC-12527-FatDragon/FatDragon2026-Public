package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.wheel.Wheel;

/**
 * Command to move the wheel to the next slot.
 */
public class WheelNextSlotCommand extends InstantCommand {
    public WheelNextSlotCommand(Wheel wheel) {
        super(wheel::nextSlot);
        addRequirements(wheel);
    }
}

