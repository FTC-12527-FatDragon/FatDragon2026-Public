package org.firstinspires.ftc.teamcode.subsystems.shooter;


import com.acmerobotics.dashboard.config.Config;

/**
 * Constants for the Shooter subsystem.
 * Defines motor names, PID coefficients, and target velocities/powers.
 */
@Config
public class ShooterConstants {
    public static String upShooterName = "upShooterMotor";
    public static String downShooterName = "downShooterMotor";

    public static double shooterEpsilon = 50; // Tolerance for velocity check

    // PID Coefficients
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
     * Target Velocities (In Ticks Per Second) for Closed Loop PID control
     */
    public static double stopVelocity = 0;
    public static double fastVelocity = -1880; // Target velocity for fast/far shots
    public static double slowVelocity = -1580; // Target velocity for slow/near shots
}
