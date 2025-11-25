package org.firstinspires.ftc.teamcode.subsystems.gimbal;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.wheel.WheelConstants;

/**
 * Gimbal Subsystem
 *
 * This subsystem controls a single servo gimbal mechanism.
 */
public class Gimbal extends SubsystemBase {
    public final Servo gimbalServo;

    /**
     * Constructor for Gimbal.
     * Initializes the gimbal servo.
     *
     * @param hardwareMap The hardware map to get the servo.
     */
    public Gimbal(HardwareMap hardwareMap){
        gimbalServo = hardwareMap.get(Servo.class, WheelConstants.wheelServoName);
    }


}
