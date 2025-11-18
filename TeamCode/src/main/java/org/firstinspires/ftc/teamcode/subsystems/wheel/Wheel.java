package org.firstinspires.ftc.teamcode.subsystems.wheel;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wheel extends SubsystemBase {

    public final Servo wheelServo;
    public final Servo upwardServo;

    public double upwardServoPosition = WheelConstants.offPosition;

    public WheelServoState wheelState = WheelServoState.IDLE;


    public Wheel(HardwareMap hardwareMap) {
        wheelServo = hardwareMap.get(Servo.class, WheelConstants.wheelServoName);

        upwardServo = hardwareMap.get(Servo.class, WheelConstants.upwardServoName);
    }

    public enum WheelServoState {
        IDLE(WheelConstants.posIDLE),
        ONE(WheelConstants.posOne),
        TWO(WheelConstants.posTwo),
        THREE (WheelConstants.posThree);

        final double servoPos;

        WheelServoState(double servoPos){
            this.servoPos = servoPos;
        }
    }

    public void setWheelState(Wheel.WheelServoState state){
        wheelState = state;
    }

    public void toggleUpwardServo() {
        upwardServoPosition = upwardServoPosition == 0.5? 1: 0.5;
    }

    @Override
    public void periodic() {
        upwardServo.setPosition(upwardServoPosition);
        wheelServo.setPosition(wheelState.servoPos);
    }
}
