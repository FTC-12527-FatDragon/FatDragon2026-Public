package org.firstinspires.ftc.teamcode.subsystems.gimbal;

import com.acmerobotics.dashboard.config.Config;

/**
 * Constants for the Gimbal subsystem.
 * Contains servo names and position values.
 */
@Config
public class GimbalConstants {
    /** Name of the gimbal servo in the hardware map */
    public static String gimbalServoName = "gimbalServo";

    // Servo positions
    public static double posIDLE = 0;
    public static double posDeg90 = 0;
    public static double posDeg180 = 0;

}
