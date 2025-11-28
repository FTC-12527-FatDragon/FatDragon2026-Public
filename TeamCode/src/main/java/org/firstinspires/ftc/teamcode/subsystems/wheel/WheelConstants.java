package org.firstinspires.ftc.teamcode.subsystems.wheel;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Config
@Configurable
public class WheelConstants {

    public static String wheelServoName = "wheelServo";
    public static String upwardServoName = "upwardServo";//continuous

    public static double offPosition = 0.2;

    public static double posIDLE = 0;
    public static double posOne = 0.5;
    public static double posTwo = 0;
    public static double posThree = 0;

    public static double upwardServoLow = 0.2;
    public static double upwardServoHigh = 0.6; // Updated to 0.6
    
    public static long launchWaitTime = 450;

    // Wheel Slots (Generated based on start 0.832, step 0.074)
    // Added 0.980 and 0.906 at the beginning.
    public static double[] WHEEL_SLOTS = {
            0.980, 0.906, 
            0.832, 0.758, 0.684, 0.610, 0.536, 0.462,
            0.388, 0.314, 0.240, 0.166, 0.092, 0.018
    };
    
    // The index of the starting position (0.832)
    public static int INITIAL_SLOT_INDEX = 2;
}
