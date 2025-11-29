package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Units;

/**
 * Constants for the Drive subsystem.
 * Includes motor names, PID coefficients, and Pedro Pathing tuning values.
 */
@Config
public class DriveConstants {
    // Motor hardware map names
    public static String leftFrontMotorName = "leftFrontMotor";
    public static String leftBackMotorName = "leftBackMotor";
    public static String rightFrontMotorName = "rightFrontMotor";
    public static String rightBackMotorName = "rightBackMotor";

    // Odometry offsets (in mm) for Pinpoint (Dead Wheels)
    // If the sensor is not at the center of the robot, measure these values.
    // Positive X is forward, Positive Y is left.
    public static double xPoseDW = 0, yPoseDW = 0;

    // Drive behavior constants
    public static double strafingBalance = 1.1; // Multiplier to correct strafing drift
    public static double headingEpsilon = 0.1; // Tolerance for heading checks (radians)
    public static DistanceUnit distanceUnit = DistanceUnit.INCH; // Changed to INCH for Pedro Pathing consistency
    public static AngleUnit angleUnit = AngleUnit.RADIANS;

    // Pedro Pathing / Tuning Constants
    public static double linearScalar = 1.00026; // Linear scaling factor for localization
    public static double angularScalar = 0.99414; // Angular scaling factor for localization
    public static double forwardVelocity = 63.966; // Max forward velocity (inches/sec)
    public static double strafeVelocity = 26.744; // Max strafe velocity (inches/sec)
    public static double forwardAcceleration = -32.6419; // Forward acceleration (inches/sec^2)
    public static double strafeAcceleration = -95.1316; // Strafe acceleration (inches/sec^2)
}
