package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.IntakeRunCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchSingleCommand;
import org.firstinspires.ftc.teamcode.commands.ResetHeadingCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterManualCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.WheelNextSlotCommand;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.commands.DriveToPoseCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.AutoPaths;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.gimbal.Gimbal;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.wheel.Wheel;
import org.firstinspires.ftc.teamcode.subsystems.wheel.WheelConstants;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;
import org.firstinspires.ftc.teamcode.commands.WheelUpwardManualCommand;

import org.firstinspires.ftc.teamcode.commands.ResetPoseCommand;

/**
 * Solo TeleOp OpMode
 *
 * This OpMode controls the robot during the driver-controlled period.
 * It initializes the drive subsystem and maps gamepad inputs to robot actions.
 */
@Config
@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpAlpha")
public class Solo extends CommandOpMode {
    private MecanumDrive drive;
    private Shooter shooter;
    private Intake intake;
    private Wheel wheel;
    private Gimbal gimbal;
    private Telemetry telemetryM;
    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    /**
     * Initializes the OpMode.
     * Sets up the hardware, gamepads, and commands.
     */
    @Override
    public void initialize() {
        drive = new MecanumDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        wheel = new Wheel(hardwareMap);
        gimbal = new Gimbal(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        // initialize Wheel position
        wheel.resetSlot();
        wheel.setUpwardServoPosition(WheelConstants.upwardServoLow);

        drive.setDefaultCommand(new TeleOpDriveCommand(drive, gamepadEx1));


        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenPressed(
                new ResetHeadingCommand(drive)
        );
        
        // Emergency Pose Reset: Gamepad 2 Left Trigger + Right Trigger + Left Bumper + Right Bumper
        new FunctionalButton(
                () -> gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5 &&
                      gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5 &&
                      gamepadEx2.getButton(GamepadKeys.Button.LEFT_BUMPER) &&
                      gamepadEx2.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ).whenPressed(
                new ResetPoseCommand(drive)
        );

        // Left Bumper: Far Shot
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)
        ).whenHeld(
                new ShooterManualCommand(shooter, ShooterConstants.shooterPowerFar)
        );

        // Right Bumper: Near Shot
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ).whenHeld(
                new ShooterManualCommand(shooter, ShooterConstants.shooterPowerNear)
        );

        // Right Trigger: Launch Single
        new FunctionalButton(
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5
        ).whenPressed(
                new LaunchSingleCommand(wheel)
        );

        // B Button: Intake Reverse
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.B)
        ).whenHeld(
                new IntakeRunCommand(intake, true)
        );

        // Left Trigger: Intake Normal
        new FunctionalButton(
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
        ).whenHeld(
                new IntakeRunCommand(intake, false)
        );

        // X Button: Next Slot
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.X)
        ).whenPressed(
                new WheelNextSlotCommand(wheel)
        );
        
        // --- Gimbal Controls ---
        
        // D-Pad Down: Auto Aim (While Held)
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN)
        ).whileHeld(
                new org.firstinspires.ftc.teamcode.commands.GimbalAutoAimCommand(gimbal, drive)
        );

        // D-Pad Up: Reset to Center (When Pressed)
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
        ).whenPressed(
                new org.firstinspires.ftc.teamcode.commands.GimbalResetCommand(gimbal)
        );

        // D-Pad Left: Manual Turn Left (While Held)
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT)
        ).whileHeld(
                new org.firstinspires.ftc.teamcode.commands.GimbalManualControlCommand(gimbal, 0.001)
        );

        // D-Pad Right: Manual Turn Right (While Held)
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT)
        ).whileHeld(
                new org.firstinspires.ftc.teamcode.commands.GimbalManualControlCommand(gimbal, -0.001)
        );

        // Test functionality: Move to NEAR_SHOT_1 when Right Stick Button is pressed
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
        ).whenPressed(
                new DriveToPoseCommand(drive, AutoPaths.NEAR_SHOT_1)
        );
    }

    /**
     * The main loop of the OpMode.
     * Runs the command scheduler and updates telemetry.
     */
    @Override
    public void run() {
        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CommandScheduler.getInstance().run();

        telemetry.addData("X (Inches)", drive.getPose().getX());
        telemetry.addData("Y (Inches)", drive.getPose().getY());
        telemetry.addData("Heading (Radians)", drive.getPose().getHeading());
        telemetry.addData("Wheel Position", wheel.customWheelPos);
        telemetry.addData("Gimbal Position", Gimbal.gimbalServoPosition);
        telemetry.update();
    }
}