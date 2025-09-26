package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "First Teleop")
public class DriveTrain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double y = 0, x = 0, pivot = 0;
        double RFPower, LFPower, RBPower, LBpower;

        DcMotor RF;
        DcMotor LF;
        DcMotor RB;
        DcMotor LB;

        RF = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        LF = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        RB = hardwareMap.get(DcMotor.class, "rightBackMotor");
        LB = hardwareMap.get(DcMotor.class, "leftBackMotor");

        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            pivot = gamepad1.right_stick_x;

            RFPower = -y - x + pivot;
            LFPower = y + x - pivot;
            RBPower = y + x + pivot;
            LBpower = -y - x - pivot;

            RF.setPower(RFPower);
            LF.setPower(LFPower);
            RB.setPower(RBPower);
            LB.setPower(LBpower);

        }

    }
}