package org.firstinspires.ftc.teamcode.subsystems.wheel;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wheel extends SubsystemBase {

    public final Servo wheelServo;

    public final Servo upwardServo;


    public wheelServoState wheelState = wheelServoState.ONE;
    public upwardServoState upwardtate = Wheel.upwardServoState.OFF;


    public Wheel(HardwareMap hardwareMap) {
        wheelServo = hardwareMap.get(Servo.class, WheelConstants.wheelServoName);

        upwardServo = hardwareMap.get(Servo.class, WheelConstants.upwardServoName);
    }

    public enum upwardServoState {
        OFF(WheelConstants.off),        // 关闭
        INTAKE(WheelConstants.intake),     // 吸入
        OUTTAKE (WheelConstants.outtake);    // 吐出

        final double setPower;

        upwardServoState(double setPower){
            this.setPower = setPower;
        }
    }

    public enum wheelServoState {
        ONE(WheelConstants.oneAngle),        //1号位
        TWO(WheelConstants.twoAngle),     // 2号位置
        THREE (WheelConstants.threeAngle);    //  3号位置

        final double setAngle;

        wheelServoState(double setAngle){
            this.setAngle = setAngle;
        }
    }

    public void setWheelStateToOne(){
        wheelState = wheelServoState.ONE;
    }

    public void setWheelStateToTwo(){
        wheelState = wheelServoState.TWO;
    }

    public void setWheelStateToThree(){
        wheelState = wheelServoState.THREE;
    }

    public void 

    @Override
    public void periodic() {

    }
}
