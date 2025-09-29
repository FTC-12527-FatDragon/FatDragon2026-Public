package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "First Teleop")
public class DriveTrain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor RF = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        DcMotor LF = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        DcMotor RB = hardwareMap.get(DcMotor.class, "rightBackMotor");
        DcMotor LB = hardwareMap.get(DcMotor.class, "leftBackMotor");

        // 右侧反转，左侧正转（常见接法）
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            double y  = -gamepad1.left_stick_y;   // 前推为正
            double x  =  gamepad1.left_stick_x;   // 右为正
            double rx =  gamepad1.right_stick_x;  // 顺时针为正

            double denom = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));

            double lf = (y + x + rx) / denom;
            double lb = (y - x + rx) / denom;
            double rf = (y - x - rx) / denom;
            double rb = (y + x - rx) / denom;

            LF.setPower(lf);
            LB.setPower(lb);
            RF.setPower(rf);
            RB.setPower(rb);

            idle();
        }
    }
}