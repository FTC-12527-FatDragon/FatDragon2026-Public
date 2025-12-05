package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

@Config
@TeleOp(name = "Motor Check (Direction & Port)", group = "Test")
public class MotorCheck extends LinearOpMode {

    DcMotor leftFront, leftBack, rightFront, rightBack;

    @Override
    public void runOpMode() {
        // Get motors directly from hardware map to test raw configuration
        leftFront = hardwareMap.get(DcMotor.class, DriveConstants.leftFrontMotorName);
        leftBack = hardwareMap.get(DcMotor.class, DriveConstants.leftBackMotorName);
        rightFront = hardwareMap.get(DcMotor.class, DriveConstants.rightFrontMotorName);
        rightBack = hardwareMap.get(DcMotor.class, DriveConstants.rightBackMotorName);

        // We DO NOT set directions here. We want to see the raw behavior.
        // Or we can set them to FORWARD to see "Raw" direction.
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Motor Check Tool");
        telemetry.addLine("Press A -> Left Front (Forward Power)");
        telemetry.addLine("Press B -> Right Front (Forward Power)");
        telemetry.addLine("Press X -> Left Back (Forward Power)");
        telemetry.addLine("Press Y -> Right Back (Forward Power)");
        telemetry.addLine("OBSERVE: Which wheel spins? Which way?");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double power = 0.3; // Low power for safety

            if (gamepad1.a) {
                leftFront.setPower(power);
                telemetry.addData("Running", "Left Front");
            } else {
                leftFront.setPower(0);
            }

            if (gamepad1.b) {
                rightFront.setPower(power);
                telemetry.addData("Running", "Right Front");
            } else {
                rightFront.setPower(0);
            }

            if (gamepad1.x) {
                leftBack.setPower(power);
                telemetry.addData("Running", "Left Back");
            } else {
                leftBack.setPower(0);
            }

            if (gamepad1.y) {
                rightBack.setPower(power);
                telemetry.addData("Running", "Right Back");
            } else {
                rightBack.setPower(0);
            }

            telemetry.update();
        }
    }
}
