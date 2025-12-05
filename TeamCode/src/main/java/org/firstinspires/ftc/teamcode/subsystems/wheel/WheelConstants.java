package org.firstinspires.ftc.teamcode.subsystems.wheel;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Config
@Configurable
public class WheelConstants {

    public static String wheelServoName = "wheelServo";
    public static String upwardServoName = "upwardServo";//continuous

    // Add Color Sensor Name
    public static String colorSensorName = "colorSensor";

    public static double offPosition = 0.2;

    public static double posIDLE = 0;
    public static double posOne = 0.5;
    public static double posTwo = 0;
    public static double posThree = 0;

    public static double upwardServoLow = 0.2;
    public static double upwardServoHigh = 0.6; // Updated to 0.6
    
    public static long launchWaitTime = 450;

    // Wheel Slots (Start 0.0, step 0.07407, max < 1.0)
    public static double[] WHEEL_SLOTS = {
            0.00000,
            0.07407,
            0.14814,
            0.22221,
            0.29628,
            0.37035,
            0.44442,
            0.51849,
            0.59256,
            0.66663,
            0.74070,
            0.81477,
            0.88884,
            0.96291
    };
    
    // The index of the starting position (Default to 0 or first slot)
    public static int INITIAL_SLOT_INDEX = 0;
    
    // Color Sensor Thresholds (Example values, tune these!)
    public static double detectionThresholdCM = 3.0; // Distance to detect a sample
    public static int blueHueThreshold = 180; // Approx hue for blue vs yellow/red
}
