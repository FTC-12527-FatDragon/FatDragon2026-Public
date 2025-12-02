package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;

/**
 * TeleOpDriveCommand
 *
 * This command handles the driving of the robot during the TeleOp period.
 * It uses field-centric driving based on gamepad input.
 */
public class TeleOpDriveCommand extends CommandBase {
    private final MecanumDrive drive;
    private final GamepadEx gamepadEx;

    /**
     * Constructor for TeleOpDriveCommand.
     *
     * @param drive     The drive subsystem.
     * @param gamepadEx The gamepad to read input from.
     */
    public TeleOpDriveCommand(MecanumDrive drive, GamepadEx gamepadEx) {
        this.drive = drive;
        this.gamepadEx = gamepadEx;
        addRequirements(drive);
    }

    /**
     * The main loop of the command.
     * Reads gamepad input and moves the robot field-centrically.
     */
    @Override
    public void execute() {
            drive.moveRobotFieldRelative(gamepadEx.getLeftY(), gamepadEx.getLeftX(), gamepadEx.getRightX());
    }
}
