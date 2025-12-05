package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

@TeleOp(name = "Test: Robot Centric Drive", group = "Test")
public class RobotCentricTest extends LinearOpMode {

    private DcMotor leftFront, leftRear, rightFront, rightRear;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Initialize Motors directly (Bypassing MecanumDrive/Follower for purity)
        leftFront = hardwareMap.get(DcMotor.class, DriveConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotor.class, DriveConstants.leftBackMotorName);
        rightFront = hardwareMap.get(DcMotor.class, DriveConstants.rightFrontMotorName);
        rightRear = hardwareMap.get(DcMotor.class, DriveConstants.rightBackMotorName);

        // 2. Set Directions based on our latest Constants.java analysis
        // LF: REV, LR: REV, RF: FWD, RR: FWD
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        // 3. Set Zero Power Behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Robot Centric Test Ready.");
        telemetry.addLine("Use Left Stick for Move, Right Stick for Turn.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 4. Gamepad Inputs
            double drive = -gamepad1.left_stick_y; // Forward (Up is -1)
            double strafe = -gamepad1.left_stick_x; // Strafe (Right is +1, Left is -1 -> want Left +)
            // Wait, standard Mecanum math usually assumes:
            // Strafe Right is Positive for math
            // Left Stick X: Right is +1. So strafe = +gamepad1.left_stick_x
            
            // Let's stick to standard FTC Robot Centric math:
            // y = -gamepad1.left_stick_y (Forward)
            // x = gamepad1.left_stick_x (Strafe Right)
            // rx = gamepad1.right_stick_x (Turn Right)
            
            double y = -gamepad1.left_stick_y; 
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // 5. Calculate Motor Powers
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            
            // Standard Mecanum Formulas (for Left=REV, Right=FWD config)
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // 6. Set Powers
            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

            // 7. Telemetry
            telemetry.addData("Inputs", "Y:%.2f, X:%.2f, R:%.2f", y, x, rx);
            telemetry.addData("Motors", "FL:%.2f, BL:%.2f, FR:%.2f, BR:%.2f",
                    frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.update();
        }
    }
}

