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
    public static double upwardServoHigh = 0.55;
}
