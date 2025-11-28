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
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveOTOS;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

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
    private MecanumDriveOTOS drive;
    private Shooter shooter;
    private Intake intake;
    private Telemetry telemetryM;
    private GamepadEx gamepadEx1;
    private boolean[] isAuto = {false};

    /**
     * Initializes the OpMode.
     * Sets up the hardware, gamepads, and commands.
     */
    @Override
    public void initialize() {
        drive = new MecanumDriveOTOS(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);


        //drive.setDefaultCommand(new TeleOpDriveCommand(drive, gamepadEx1, isAuto));


//        new FunctionalButton(
//                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
//        ).whenPressed(
//                new InstantCommand(() -> drive.reset(0))
//        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.A)
        ).whenHeld(
                new InstantCommand(() -> shooter.setOpenLoopPower(0.7))
        ).whenReleased(
                new InstantCommand(() -> shooter.setOpenLoopPower(0))
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.B)
        ).whenHeld(
                new InstantCommand(() -> intake.setRunning(true))
        ).whenReleased(
                new InstantCommand(() -> intake.setRunning(false))
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

        telemetry.addData("X", drive.getPose().getX(DistanceUnit.MM));
        telemetry.addData("Y", drive.getPose().getY(DistanceUnit.MM));
        telemetry.addData("Heading", drive.getPose().getHeading(AngleUnit.RADIANS));
        telemetry.addData("YawOffset", drive.getYawOffset());
        telemetry.update();


    }
}