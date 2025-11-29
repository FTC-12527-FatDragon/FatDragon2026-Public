package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * AutoPaths - A library of Poses and PathChains for Autonomous and TeleOp navigation.
 */
public class AutoPaths {

    // --- Poses (Key Field Positions) ---
    // Format: x (inches), y (inches), heading (radians)
    
    // Blue Alliance Start Poses
    public static final Pose BLUE_START_1 = new Pose(47.071, 11.481, Math.toRadians(90));
    public static final Pose BLUE_START_2 = new Pose(27.390, 131.043, Math.toRadians(324));
    
    // Red Alliance Start Poses
    public static final Pose RED_START_1 = new Pose(94.141, 11.645, Math.toRadians(90));
    public static final Pose RED_START_2 = new Pose(114.150, 131.043, Math.toRadians(216));

    // --- Scoring Targets (Basket Locations) ---
    // Note: These are likely the coordinates of the baskets themselves, used for distance calculation.
    // For navigation, you usually want a pose *facing* the basket, a few inches away.
    public static final Pose BLUE_BASKET = new Pose(11.317, 135.636, 0); 
    public static final Pose RED_BASKET = new Pose(128.583, 11.317, 0);

    // --- Shooting Poses ---
    // Pre-defined spots for shooting. Heading is set to 0 by default, adjust to face basket if needed.
    public static final Pose NEAR_SHOT_1 = new Pose(70, 73, 0);
    public static final Pose FAR_SHOT_1 = new Pose(71, 50, 0);

    // --- Path Builders ---
    
    /**
     * Path: From Blue Start 1 to Near Shot 1.
     * A simple straight line path.
     */
    public static PathChain blueStartToNearShot(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierLine(BLUE_START_1, NEAR_SHOT_1))
                .setLinearHeadingInterpolation(BLUE_START_1.getHeading(), NEAR_SHOT_1.getHeading())
                .build();
    }

    // --- Utility Methods ---

    /**
     * Calculates the angle difference between the robot's current heading and the target basket.
     * Positive value means turn LEFT (Counter-Clockwise).
     * Negative value means turn RIGHT (Clockwise).
     *
     * @param robotPose The current pose of the robot (from follower.getPose()).
     * @param basketPose The target basket pose (e.g., BLUE_BASKET).
     * @return The angle difference in Radians, normalized to [-PI, PI].
     */
    public static double getAngleToBasket(Pose robotPose, Pose basketPose) {
        double dx = basketPose.getX() - robotPose.getX();
        double dy = basketPose.getY() - robotPose.getY();
        double targetHeading = Math.atan2(dy, dx);

        // Calculate difference
        double angleDiff = targetHeading - robotPose.getHeading();

        // Normalize to [-PI, PI]
        while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
        while (angleDiff <= -Math.PI) angleDiff += 2 * Math.PI;

        return angleDiff;
    }
}
