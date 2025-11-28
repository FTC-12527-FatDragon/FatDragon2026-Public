package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.wheel.Wheel;
import org.firstinspires.ftc.teamcode.subsystems.wheel.WheelConstants;

public class LaunchSingleCommand extends SequentialCommandGroup {

    public LaunchSingleCommand(Wheel wheel) {
        addRequirements(wheel);
        addCommands(
            new InstantCommand(() -> wheel.setUpwardServoPosition(WheelConstants.upwardServoHigh)),
            new WaitCommand(WheelConstants.launchWaitTime),
            new InstantCommand(() -> wheel.setUpwardServoPosition(WheelConstants.upwardServoLow))
        );
    }
}
