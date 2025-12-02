package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.commands.IntakeRunCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchSingleCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterManualCommand;
import org.firstinspires.ftc.teamcode.commands.WheelNextSlotCommand;
import org.firstinspires.ftc.teamcode.commands.WheelUpwardManualCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.gimbal.Gimbal;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.wheel.Wheel;
import org.firstinspires.ftc.teamcode.subsystems.wheel.WheelConstants;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

/**
 * SoloRobotCentric TeleOp OpMode
 * 
 * A simplified "Headed" (Robot-Centric) TeleOp that does NOT use Odometry (Pinpoint/OTOS).
 * Use this as a fallback if the localization sensor fails.
 */
@Config
@TeleOp(name = "Solo-RobotCentric-NoOD", group = "Fallback")
public class SoloRobotCentric extends CommandOpMode {

    // Direct Motor Access (No MecanumDrive Subsystem to avoid Follower/OD dependency)
    private DcMotor leftFront, leftRear, rightFront, rightRear;

    private Shooter shooter;
    private Intake intake;
    private Wheel wheel;
    private Gimbal gimbal;
    private MultipleTelemetry telemetryM;
    private GamepadEx gamepadEx1;

    @Override
    public void initialize() {
        // --- Initialize Drive Motors Directly ---
        leftFront = hardwareMap.get(DcMotor.class, DriveConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotor.class, DriveConstants.leftBackMotorName);
        rightFront = hardwareMap.get(DcMotor.class, DriveConstants.rightFrontMotorName);
        rightRear = hardwareMap.get(DcMotor.class, DriveConstants.rightBackMotorName);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Initialize Subsystems ---
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        wheel = new Wheel(hardwareMap);
        gimbal = new Gimbal(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);

        // --- Initialize Subsystem States ---
        wheel.resetSlot();
        wheel.setUpwardServoPosition(WheelConstants.upwardServoLow);

        // --- Drive Control (Robot Centric) ---
        // Using a RunCommand that runs every loop to set motor powers
        schedule(new RunCommand(() -> {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Robot Centric Math
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);
        }));

        // --- Controls (Copied from Solo.java) ---
        
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

        // D-Pad Down: Next Slot - REPLACED BY X BUTTON
//        new FunctionalButton(
//                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN)
//        ).whenPressed(
//                new WheelNextSlotCommand(wheel)
//        );

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

        // X Button: Upward Servo Manual
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.X)
        ).whenPressed(
                new WheelNextSlotCommand(wheel)
        );
        
        // Note: ResetHeading and DriveToPose are removed because they require OD.
    }

    @Override
    public void run() {
        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Gimbal Servo Manual Control
        if (gamepad1.dpad_left) {
            Gimbal.gimbalServoPosition += 0.001;
            gimbal.setGimbalState(Gimbal.GimbalServoState.AIM);
        } else if (gamepad1.dpad_right) {
            Gimbal.gimbalServoPosition -= 0.001;
            gimbal.setGimbalState(Gimbal.GimbalServoState.AIM);
        }

        CommandScheduler.getInstance().run();

        telemetry.addData("Mode", "Robot Centric (No OD)");
        telemetry.addData("Wheel Position", wheel.customWheelPos);
        telemetry.addData("Gimbal Position", Gimbal.gimbalServoPosition);
        telemetry.update();
    }
}

