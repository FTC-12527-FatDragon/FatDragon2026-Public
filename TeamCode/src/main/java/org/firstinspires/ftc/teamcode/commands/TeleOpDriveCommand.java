package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveOTOS;

/**
 * TeleOpDriveCommand
 *
 * This command handles the driving of the robot during the TeleOp period.
 * It uses field-centric driving based on gamepad input.
 */
public class TeleOpDriveCommand extends CommandBase {
    private final MecanumDrive drive;
    private final GamepadEx gamepadEx;
    private final boolean[] isAuto;

    /**
     * Constructor for TeleOpDriveCommand.
     *
     * @param drive     The drive subsystem.
     * @param gamepadEx The gamepad to read input from.
     * @param isAuto    A boolean array indicating if an autonomous action is currently taking control.
     */
    public TeleOpDriveCommand(MecanumDrive drive, GamepadEx gamepadEx, boolean[] isAuto) {
        this.drive = drive;
        this.gamepadEx = gamepadEx;
        this.isAuto = isAuto;
        addRequirements(drive);
    }

    /**
     * The main loop of the command.
     * Reads gamepad input and moves the robot field-centrically if not in auto mode.
     */
    @Override
    public void execute() {
        if (!isAuto[0]) {
            drive.moveRobotFieldRelative(gamepadEx.getLeftY(), gamepadEx.getLeftX(), gamepadEx.getRightX());
        }
    }
}
