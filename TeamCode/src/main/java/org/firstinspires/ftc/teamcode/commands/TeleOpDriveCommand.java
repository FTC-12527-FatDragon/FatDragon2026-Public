package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Drive.MecanumDrive;


public class TeleOpDriveCommand extends CommandBase {
    private final MecanumDrive drive;
    private final GamepadEx gamepadEx;
    private final boolean[] isAuto;

    public TeleOpDriveCommand(MecanumDrive drive, GamepadEx gamepadEx, boolean[] isAuto) {
        this.drive = drive;
        this.gamepadEx = gamepadEx;
        this.isAuto = isAuto;
        addRequirements(drive);
    }

    public TeleOpDriveCommand(MecanumDrive drive, GamepadEx gamepadEx1, boolean[] isAuto, MecanumDrive drive1, GamepadEx gamepadEx, boolean[] isAuto1) {
        this.drive = drive1;
        this.gamepadEx = gamepadEx;
        this.isAuto = isAuto1;
    }

    @Override
    public void execute() {
        if (!isAuto[0]) {
            drive.moveRobotFieldRelative(-gamepadEx.getLeftY(), -gamepadEx.getLeftX(), gamepadEx.getRightX());
        }
    }
}
