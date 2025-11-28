package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;

/**
 * Command to reset the robot's heading to zero.
 */
public class ResetHeadingCommand extends InstantCommand {
    public ResetHeadingCommand(MecanumDrive drive) {
        super(() -> drive.reset(0));
    }
}

