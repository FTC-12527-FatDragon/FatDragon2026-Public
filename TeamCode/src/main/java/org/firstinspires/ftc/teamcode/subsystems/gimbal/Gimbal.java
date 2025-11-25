package org.firstinspires.ftc.teamcode.subsystems.gimbal;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Gimbal Subsystem
 *
 * This subsystem controls a single servo gimbal mechanism used to aim or position elements.
 * It supports preset positions (IDLE, 90, 180) and a custom AIM position.
 */
public class Gimbal extends SubsystemBase {

    public final Servo gimbalServo;
    public static double gimbalServoPosition = GimbalConstants.posIDLE;

    public GimbalServoState gimbalState = GimbalServoState.IDLE;

    /**
     * Constructor for Gimbal.
     * Initializes the gimbal servo.
     *
     * @param hardwareMap The hardware map to get the servo.
     */
    public Gimbal(HardwareMap hardwareMap) {
        gimbalServo = hardwareMap.get(Servo.class, GimbalConstants.gimbalServoName);
    }

    public enum GimbalServoState {
        IDLE(GimbalConstants.posIDLE),
        DEG90(GimbalConstants.posDeg90),
        DEG180(GimbalConstants.posDeg180),
        AIM(gimbalServoPosition);

        final double servoPos;

        GimbalServoState(double servoPos){
            this.servoPos = servoPos;
        }
    }

    public void setGimbalState(Gimbal.GimbalServoState state){
        gimbalState = state;
    }

    @Override
    public void periodic() {
        if (gimbalState != GimbalServoState.AIM) gimbalServo.setPosition(gimbalState.servoPos);
        else gimbalServo.setPosition(gimbalServoPosition);
    }
}
