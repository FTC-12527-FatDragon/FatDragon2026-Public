package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * Standard Pedro Pathing Mecanum Drive Subsystem.
 * Wraps the Follower class for Command-Based programming.
 */
@Config
public class MecanumDrive extends SubsystemBase {
    public final Follower follower;
    private Telemetry telemetry;

    public MecanumDrive(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        
        // Set motors to BRAKE mode for active braking (Idle Brake)
        try {
            hardwareMap.get(DcMotor.class, DriveConstants.leftFrontMotorName).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardwareMap.get(DcMotor.class, DriveConstants.leftBackMotorName).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardwareMap.get(DcMotor.class, DriveConstants.rightFrontMotorName).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardwareMap.get(DcMotor.class, DriveConstants.rightBackMotorName).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
        }
        
        // Optimization: Bulk Caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        
        // Initialize TeleOp Drive
        follower.setStartingPose(PoseStorage.currentPose);
        follower.startTeleopDrive();
    }
    
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        follower.update();
    }

    /**
     * Standard Field Centric Drive.
     */
    public void moveRobotFieldRelative(double forward, double strafe, double turn) {
        // Gamepad Inputs (from TeleOpDriveCommand):
        // forward: Negative for Forward
        // strafe: Negative for Left
        // turn: Negative for CCW (assuming cubicScale maintains sign)
        
        // We want positive values for +X (Forward), +Y (Left), +Rot (CCW)
        double xField = forward;
        double yField = -strafe;
        double rot = -turn;

        double heading = follower.getPose().getHeading();
        
        // DEBUG: Check if heading is actually updating
        if (telemetry != null) {
            telemetry.addData("Drive/Heading", Math.toDegrees(heading));
            telemetry.addData("Drive/InputX", xField);
            telemetry.addData("Drive/InputY", yField);
        }

        // Standard Rotation Matrix for Field Centric
        // x_robot = x_field * cos(theta) + y_field * sin(theta)
        // y_robot = -x_field * sin(theta) + y_field * cos(theta)
        
        // NOTE: If Pinpoint heading is inverted (CW+), use -heading
        // Assuming Pinpoint is CCW+ (Standard)
        
        double xRobot = xField * Math.cos(heading) + yField * Math.sin(heading);
        double yRobot = -xField * Math.sin(heading) + yField * Math.cos(heading);
        
        follower.setTeleOpDrive(xField, yField, rot, false);
    }

    public void moveRobot(double forward, double strafe, double turn) {
        follower.setTeleOpDrive(-forward, -strafe, -turn, false);
    }
    
    public void reset(double heading) {
        Pose current = follower.getPose();
        follower.setPose(new Pose(current.getX(), current.getY(), heading));
    }

    public Pose getPose() {
        return follower.getPose();
    }
}
