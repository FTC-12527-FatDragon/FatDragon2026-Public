package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;

/**
 * TeleOpDriveCommand
 *
 * This command handles the driving of the robot during the TeleOp period.
 * It uses field-centric driving based on gamepad input with non-linear (exponential) mapping.
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
        // Apply non-linear mapping for better control at low speeds
        double forward = cubicScale(gamepadEx.getLeftY());
        double strafe = cubicScale(gamepadEx.getLeftX());
        double turn = cubicScale(gamepadEx.getRightX());

        drive.moveRobotFieldRelative(forward, strafe, turn);
    }
    
    /**
     * Scales the input using a cubic function (x^3) to preserve sign and allow fine control.
     * 
     * @param input The raw gamepad input (-1.0 to 1.0).
     * @return The scaled output.
     */
    private double cubicScale(double input) {
        return Math.pow(input, 3);
    }
}
