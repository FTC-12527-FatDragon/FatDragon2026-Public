package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.strafingBalance;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Util;

/**
 * MecanumDrive Subsystem (OTOS Version)
 *
 * This subsystem controls the robot's drivetrain using mecanum wheels and the SparkFun OTOS sensor for localization.
 * It supports both field-centric and robot-centric driving.
 */
@Config
public class MecanumDrive extends SubsystemBase {
    private final DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    private final SparkFunOTOS otos;
    private double yawOffset;
    public static double xPose = DriveConstants.xPoseOTOS, yPose = DriveConstants.yPoseOTOS,
            headingPose = DriveConstants.headingPoseOTOS; // mm

    /**
     * Constructor for MecanumDrive.
     * Initializes motors and the OTOS sensor.
     *
     * @param hardwareMap The hardware map to get the motors and sensor.
     */
    public MecanumDrive(final HardwareMap hardwareMap) {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        otos.resetTracking();
        otos.setAngularUnit(DriveConstants.angleUnit);
        otos.setLinearUnit(DriveConstants.distanceUnit);
        otos.setOffset(new SparkFunOTOS.Pose2D(xPose, yPose,headingPose));

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Resets the robot's heading.
     *
     * @param heading The new heading to set (in radians).
     */
    public void reset(double heading) {
        yawOffset = otos.getPosition().h + heading;
    }

    /**
     * Moves the robot relative to the field (Field-Centric Drive).
     *
     * @param forward Forward speed.
     * @param fun     Strafe speed (sideways).
     * @param turn    Rotation speed.
     */
    public void moveRobotFieldRelative(double forward, double fun, double turn) {

        double botHeading = otos.getPosition().h - yawOffset;
        // Rotate the movement direction counter to the bot's rotation\\
        double rotX = fun * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = fun * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        rotX = rotX * strafingBalance; // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double leftFrontPower = (rotY + rotX + turn) / denominator;
        double leftBackPower = (rotY - rotX + turn) / denominator;
        double rightFrontPower = (rotY - rotX - turn) / denominator;
        double rightBackPower = (rotY + rotX - turn) / denominator;

        leftFrontMotor.setPower(leftFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);
    }

    /**
     * Moves the robot relative to itself (Robot-Centric Drive).
     *
     * @param forward Forward speed.
     * @param fun     Strafe speed.
     * @param turn    Rotation speed.
     */
    public void moveRobot(double forward, double fun, double turn) {
        double rotX = fun * strafingBalance; // Counteract imperfect strafing
        double rotY = forward;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double leftFrontPower = (rotY + rotX + turn) / denominator;
        double leftBackPower = (rotY - rotX + turn) / denominator;
        double rightFrontPower = (rotY - rotX - turn) / denominator;
        double rightBackPower = (rotY + rotX - turn) / denominator;

        leftFrontMotor.setPower(leftFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);
    }

    /**
     * Turns the robot by a relative angle using a simple P control loop (blocking).
     *
     * @param angle The angle to turn (relative).
     * @param power The power to turn at.
     */
    public void turnRobot(double angle, double power) {
        double preAngle = otos.getPosition().h;
        while (otos.getPosition().h - preAngle < angle) {
            leftFrontMotor.setPower(power * 0.2);
            leftBackMotor.setPower(power * 0.2);
            rightFrontMotor.setPower(power * -0.2);
            rightBackMotor.setPower(power * -0.2);
        }
    }

    /**
     * Turns the robot to an absolute heading using a simple P control loop (blocking).
     *
     * @param angle The target absolute heading.
     * @param power The power to turn at.
     */
    public void turnRobotTo(double angle, double power) {
        double heading = otos.getPosition().h;
        double needs = (angle - heading) % (2 * Math.PI);
        if(0 <= needs && needs <= Math.PI || needs <= -Math.PI)
            while (Util.epsilonEqual(angle,otos.getPosition().h,0.02)) {
                leftFrontMotor.setPower(power * 0.2);
                leftBackMotor.setPower(power * 0.2);
                rightFrontMotor.setPower(power * -0.2);
                rightBackMotor.setPower(power * -0.2);
            }
    }

    /**
     * Gets the current pose of the robot from the OTOS sensor.
     *
     * @return The current Pose2D.
     */
    public Pose2D getPose() {
        SparkFunOTOS.Pose2D pose = otos.getPosition();
        return new Pose2D(DriveConstants.distanceUnit, pose.x, pose.y, DriveConstants.angleUnit, pose.h);
    }

    public double getYawOffset() {return yawOffset;}

    public boolean isHeadingAtSetPoint(double headingSetPoint) {
        return Util.epsilonEqual(otos.getPosition().h, headingSetPoint,
                DriveConstants.headingEpsilon);
    }

    /**
     * Stops the robot.
     */
    public void stop() {
        moveRobot(0, 0, 0);
    }
}

