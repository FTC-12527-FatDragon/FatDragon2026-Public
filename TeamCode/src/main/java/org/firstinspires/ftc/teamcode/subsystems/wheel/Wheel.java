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

    public WheelServoState wheelState = WheelServoState.ONE;


    public Wheel(HardwareMap hardwareMap) {
        wheelServo = hardwareMap.get(Servo.class, WheelConstants.wheelServoName);

        upwardServo = hardwareMap.get(Servo.class, WheelConstants.upwardServoName);
    }

    public enum WheelServoState {
        IDLE(WheelConstants.idleAngle),
        ONE(WheelConstants.oneAngle),        //1
        TWO(WheelConstants.twoAngle),     //2
        THREE (WheelConstants.threeAngle);    //3

        final double setAngle;

        WheelServoState(double setAngle){
            this.setAngle = setAngle;
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
        
    }
}
