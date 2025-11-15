package org.firstinspires.ftc.teamcode.subsystems.wheel;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Config
@Configurable
public class WheelConstants {

    public static String wheelServoName = "wheelServo";
    public static String upwardServoName = "upwardServo";//连续舵机

    public static double off = 0.5;
    public static double intake = 1;
    public static double outtake = 0.5;

    public static double oneAngle = 0.5;
    public static double twoAngle = 0;
    public static double threeAngle = 0;

    public static double wheelServoState[] = {1 ,2 ,3};
}
