package org.firstinspires.ftc.teamcode.subsystems.gimbal;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.wheel.WheelConstants;

public class Gimbal extends SubsystemBase {
    public final Servo gimbalServo;

    public Gimbal(HardwareMap hardwareMap){
        gimbalServo = hardwareMap.get(Servo.class, WheelConstants.wheelServoName);
    }


}
