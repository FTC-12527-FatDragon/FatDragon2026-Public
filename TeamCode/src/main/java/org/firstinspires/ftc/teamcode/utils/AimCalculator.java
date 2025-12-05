package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.geometry.Pose;

public class AimCalculator {

    // Shooter Offset from Robot Center (in Inches)
    // NOTE: Check your coordinate system! 
    // PedroPathing/Standard: X is Forward, Y is Left.
    // If shooter is 79mm FORWARD from center, set X = 79/25.4, Y = 0.
    // If shooter is 79mm LEFT from center, set X = 0, Y = 79/25.4.
    // Based on user input "center distance to chassis center +y 79mm", I assume Y offset.
    // BUT usually shooters are offset in X (Forward). Please verify!
    // I will set Y offset as requested, but keep X available.
    public static double SHOOTER_OFFSET_X_INCH = 0.0; 
    public static double SHOOTER_OFFSET_Y_INCH = 79.0 / 25.4; // ~3.11 inches

    /**
     * Calculates the servo position (0.0 - 1.0) for a 4.5-turn servo with a 4:1 external gear ratio.
     * 
     * Logic:
     * 1. Calculate absolute position of the shooter/turret.
     * 2. Calculate angle from shooter to target.
     * 3. Multiply by 4 (Gear Ratio).
     * 4. Convert angle difference to servo value difference.
     * 5. Add to the CENTER servo value (assumed 0.5).
     *
     * @param robotPose The current pose of the robot.
     * @param targetPose The target pose to aim at.
     * @return The ABSOLUTE servo position to set.
     */
    public static double getAutoAimServoPos(Pose robotPose, Pose targetPose) {
        // 1. Calculate angle difference (Target - TurretHeading) in degrees
        double relativeAngleDeg = Math.toDegrees(getAngleToTarget(robotPose, targetPose));

        // 2. Apply Gear Ratio (4:1)
        // If turret needs to turn 90 deg, servo needs to turn 360 deg.
        double servoNeededAngle = relativeAngleDeg * 4.0;

        // 3. Convert angle to servo value (0-1)
        // Total range of servo is 1620 degrees (4.5 turns)
        // Value change per degree = 1 / 1620 ≈ 0.000617
        double servoValueDelta = servoNeededAngle / 1620.0;

        // 4. Apply to Center Value (Assumed 0.5 is Forward)
        // Note: You might need to tune 0.5 to your actual center.
        double centerValue = 0.5;
        
        return centerValue + servoValueDelta;
    }

    /**
     * Calculates the angle difference between the robot's current heading and the target.
     * Accounts for the Shooter's physical offset from the robot center.
     * 
     * @param robotPose The current pose of the robot center.
     * @param targetPose The target pose.
     * @return The angle difference in Radians, normalized to [-PI, PI].
     */
    public static double getAngleToTarget(Pose robotPose, Pose targetPose) {
        // Calculate Turret Absolute Position
        double theta = robotPose.getHeading();
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);

        // Transform Offset from Robot Frame to Field Frame
        // x_field = x_robot * cos - y_robot * sin
        // y_field = x_robot * sin + y_robot * cos
        double dx_offset = SHOOTER_OFFSET_X_INCH * cos - SHOOTER_OFFSET_Y_INCH * sin;
        double dy_offset = SHOOTER_OFFSET_X_INCH * sin + SHOOTER_OFFSET_Y_INCH * cos;

        double turretX = robotPose.getX() + dx_offset;
        double turretY = robotPose.getY() + dy_offset;

        // Calculate vector from Turret to Target
        double dx = targetPose.getX() - turretX;
        double dy = targetPose.getY() - turretY;
        
        double targetHeading = Math.atan2(dy, dx);

        double angleDiff = targetHeading - robotPose.getHeading();

        while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
        while (angleDiff <= -Math.PI) angleDiff += 2 * Math.PI;

        return angleDiff;
    }
}

