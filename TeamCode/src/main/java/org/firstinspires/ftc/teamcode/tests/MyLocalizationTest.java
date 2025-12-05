package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;

import java.util.ArrayList;

@Config
@TeleOp(name = "Test: Localization (Custom)", group = "Test")
public class MyLocalizationTest extends LinearOpMode {
    
    private MecanumDrive drive;
    private final ArrayList<Pose> pathHistory = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Hardware
        drive = new MecanumDrive(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Reset Pose
        drive.follower.setPose(new Pose(0, 0, 0));

        telemetry.addLine("Localization Test");
        telemetry.addLine("Push robot and check Dashboard Graph/Field.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.periodic(); 
            
            // Simple Manual Control (Check if sticks move the robot field-centric)
            drive.moveRobotFieldRelative(
                -gamepad1.left_stick_y, 
                -gamepad1.left_stick_x, 
                -gamepad1.right_stick_x
            );

            Pose currentPose = drive.getPose();
            pathHistory.add(currentPose);
            if (pathHistory.size() > 2000) pathHistory.remove(0);

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            drawRobot(fieldOverlay, currentPose);
            
            // Draw Path
            if (pathHistory.size() > 1) {
                double[] xPoints = new double[pathHistory.size() / 5 + 1];
                double[] yPoints = new double[pathHistory.size() / 5 + 1];
                int idx = 0;
                for (int i = 0; i < pathHistory.size(); i += 5) {
                    xPoints[idx] = pathHistory.get(i).getX();
                    yPoints[idx] = pathHistory.get(i).getY();
                    idx++;
                }
                fieldOverlay.setStroke("green");
                fieldOverlay.strokePolyline(xPoints, yPoints);
            }

            packet.put("X", currentPose.getX());
            packet.put("Y", currentPose.getY());
            packet.put("H_Deg", Math.toDegrees(currentPose.getHeading()));
            
            dashboard.sendTelemetryPacket(packet);
        }
    }
    
    private void drawRobot(Canvas canvas, Pose pose) {
        canvas.setStroke("blue");
        double h = pose.getHeading();
        double cos = Math.cos(h), sin = Math.sin(h);
        // 14 inch size
        double r = 7.0; 
        
        // Corners
        double[] x = new double[5];
        double[] y = new double[5];
        
        // FL, FR, BR, BL, FL
        double[] dx = {r, r, -r, -r, r}; // Front is +X? Pedro usually X is forward.
        // Wait, for box drawing:
        // X is forward/back, Y is left/right relative to robot
        
        // Let's assume standard box aligned to robot axes
        // Then rotate
        // FL: x=r, y=r ? No, if Width=Length.
        
        // Simple rotated box
        // FL (Front Left): X+, Y+
        // FR (Front Right): X+, Y-
        // BR (Back Right): X-, Y-
        // BL (Back Left): X-, Y+
        
        // NOTE: Pedro coords: X Forward, Y Left.
        
        double[] cx = {r, r, -r, -r, r};
        double[] cy = {r, -r, -r, r, r};
        
        for(int i=0; i<5; i++) {
            x[i] = pose.getX() + (cx[i] * cos - cy[i] * sin);
            y[i] = pose.getY() + (cx[i] * sin + cy[i] * cos);
        }
        
        canvas.strokePolyline(x, y);
        
        // Front Marker (Red line)
        canvas.setStroke("red");
        double fx = pose.getX() + r * cos;
        double fy = pose.getY() + r * sin;
        canvas.strokeLine(pose.getX(), pose.getY(), fx, fy);
    }
}