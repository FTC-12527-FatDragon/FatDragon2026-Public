package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.IntakeSmartCommand;
import org.firstinspires.ftc.teamcode.commands.ResetHeadingCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.WheelNextSlotCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.wheel.Wheel;
import org.firstinspires.ftc.teamcode.subsystems.wheel.WheelConstants;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

/**
 * Test OpMode for Color Sensor Integrated Intake.
 * 
 * Controls:
 * - Left Stick: Drive (Field Centric)
 * - Right Stick: Turn
 * - Left Stick Button: Reset Heading
 * - X: Next Wheel Slot
 * - Left Trigger: Smart Intake (Stops if sample detected)
 * - B: Reverse Intake (Always works)
 */
@TeleOp(name = "Test: Color Intake Smart")
public class ColorIntakeTest extends CommandOpMode {
    private MecanumDrive drive;
    private Intake intake;
    private Wheel wheel;
    private GamepadEx gamepadEx1;

    @Override
    public void initialize() {
        drive = new MecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        wheel = new Wheel(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);

        // Init Wheel
        wheel.resetSlot();
        wheel.setUpwardServoPosition(WheelConstants.upwardServoLow);

        // Drive
        drive.setDefaultCommand(new TeleOpDriveCommand(drive, gamepadEx1));
        
        new FunctionalButton(() -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON))
                .whenPressed(new ResetHeadingCommand(drive));

        // Wheel Slot
        new FunctionalButton(() -> gamepadEx1.getButton(GamepadKeys.Button.X))
                .whenPressed(new WheelNextSlotCommand(wheel));

        // Smart Intake (LT)
        new FunctionalButton(() -> gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenHeld(new IntakeSmartCommand(intake, wheel, false));

        // Reverse Intake (B) - Always allowed
        new FunctionalButton(() -> gamepadEx1.getButton(GamepadKeys.Button.B))
                .whenHeld(new IntakeSmartCommand(intake, wheel, true));
    }

    @Override
    public void run() {
        super.run();
        
        // Telemetry
        telemetry.addData("Has Sample?", wheel.hasSample());
        telemetry.addData("Sample Color", wheel.getSampleColor());
        telemetry.addData("Wheel Slot", wheel.currentSlotIndex);
        telemetry.update();
    }
}

