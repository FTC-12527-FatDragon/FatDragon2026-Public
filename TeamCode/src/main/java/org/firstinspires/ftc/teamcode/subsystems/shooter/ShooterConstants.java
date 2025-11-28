package org.firstinspires.ftc.teamcode.subsystems.shooter;


import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static String upShooterName = "upShooterMotor";
    public static String downShooterName = "downShooterMotor";

    public static double shooterEpsilon = 50;

    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 0;
    public static double stopPower = 0;
    
    // Power levels for Open Loop control
    public static double shooterPowerNear = 0.575; // Near Shot
    public static double shooterPowerFar = 0.8;    // Far Shot

    public static double slowPower = 0.575;
    public static double fastPower = 0.8;

    /**
     * In Ticks Per Second
     */
    public static double stopVelocity = 0;
    public static double fastVelocity = -1880;
    public static double slowVelocity = -1580;
}
