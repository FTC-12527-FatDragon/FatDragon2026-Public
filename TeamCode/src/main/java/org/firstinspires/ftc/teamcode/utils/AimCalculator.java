package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.geometry.Pose;

public class AimCalculator {

    /**
     * Calculates the servo position (0.0 - 1.0) for a 4.5-turn servo with a 4:1 external gear ratio.
     * 
     * Logic:
     * 1. Calculate angle from robot head to target.
     * 2. Multiply by 4 (Gear Ratio).
     * 3. Convert angle difference to servo value difference.
     * 4. Add to the CENTER servo value (assumed 0.5).
     *
     * @param robotPose The current pose of the robot.
     * @param targetPose The target pose to aim at.
     * @return The ABSOLUTE servo position to set.
     */
    public static double getAutoAimServoPos(Pose robotPose, Pose targetPose) {
        // 1. Calculate angle difference (Target - RobotHead) in degrees
        // Positive = Target is to the LEFT
        // Negative = Target is to the RIGHT
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
     * 
     * @param robotPose The current pose of the robot.
     * @param targetPose The target pose.
     * @return The angle difference in Radians, normalized to [-PI, PI].
     */
    public static double getAngleToTarget(Pose robotPose, Pose targetPose) {
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        double targetHeading = Math.atan2(dy, dx);

        double angleDiff = targetHeading - robotPose.getHeading();

        while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
        while (angleDiff <= -Math.PI) angleDiff += 2 * Math.PI;

        return angleDiff;
    }
}

