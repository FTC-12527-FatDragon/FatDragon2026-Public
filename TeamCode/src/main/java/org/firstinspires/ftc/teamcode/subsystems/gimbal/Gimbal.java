package org.firstinspires.ftc.teamcode.subsystems.gimbal;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gimbal extends SubsystemBase {

    public final Servo gimbalServo;

    public static double gimbalServoPosition = GimbalConstants.posIDLE;

    public GimbalServoState gimbalState = GimbalServoState.IDLE;


    public Gimbal(HardwareMap hardwareMap) {
        gimbalServo = hardwareMap.get(Servo.class, GimbalConstants.gimbalServoName);
    }

    public enum GimbalServoState {
        IDLE(GimbalConstants.posIDLE),
        DEG90(GimbalConstants.posDeg90),
        DEG180(GimbalConstants.posDeg180),
        AIM(gimbalServoPosition);

        double servoPos;

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
