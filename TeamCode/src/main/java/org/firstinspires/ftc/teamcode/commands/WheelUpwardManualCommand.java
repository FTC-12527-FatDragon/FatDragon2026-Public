package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.wheel.Wheel;
import org.firstinspires.ftc.teamcode.subsystems.wheel.WheelConstants;

/**
 * Command to manually control the Upward Servo.
 * Raises to HIGH when initialized, lowers to LOW when ended.
 */
public class WheelUpwardManualCommand extends CommandBase {
    private final Wheel wheel;

    public WheelUpwardManualCommand(Wheel wheel) {
        this.wheel = wheel;
        addRequirements(wheel);
    }

    @Override
    public void initialize() {
        wheel.setUpwardServoPosition(WheelConstants.upwardServoHigh);
    }

    @Override
    public void end(boolean interrupted) {
        wheel.setUpwardServoPosition(WheelConstants.upwardServoLow);
    }
}

