package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;

/**
 * Constants for the Intake subsystem.
 * Defines motor names and power levels.
 */
@Config
public class IntakeConstants {
    /** Name of the intake motor in the hardware map */
    public static String intakeMotorName = "intakeMotor";

    /** Power level for normal intake operation (0.0 to 1.0) */
    public static double intakePower = 1.0;
    
    /** Power level for reversing the intake (should be negative) */
    public static double reversedPower = -0.7;
}
