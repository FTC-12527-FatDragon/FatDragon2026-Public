package org.firstinspires.ftc.teamcode.subsystems.wheel;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Wheel Subsystem
 *
 * This subsystem controls the "Wheel" mechanism, which includes a servo for rotating the wheel
 * and another servo for moving it up and down.
 */
public class Wheel extends SubsystemBase {

    public final Servo wheelServo;
    public final Servo upwardServo;

    public double upwardServoPosition = WheelConstants.offPosition;

    public WheelServoState wheelState = WheelServoState.ONE;


    /**
     * Constructor for Wheel.
     * Initializes the servos.
     *
     * @param hardwareMap The hardware map to get the servos.
     */
    public Wheel(HardwareMap hardwareMap) {
        wheelServo = hardwareMap.get(Servo.class, WheelConstants.wheelServoName);

        upwardServo = hardwareMap.get(Servo.class, WheelConstants.upwardServoName);
    }

    /**
     * Enum for the different states of the wheel servo.
     */
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

    /**
     * Sets the state of the wheel servo.
     *
     * @param state The target state.
     */
    public void setWheelState(Wheel.WheelServoState state){
        wheelState = state;
    }

    /**
     * Toggles the upward servo position between two preset values.
     */
    public void toggleUpwardServo() {
        upwardServoPosition = upwardServoPosition == 0.5? 1: 0.5;
    }

    /**
     * Periodic method called by the CommandScheduler.
     * Updates the upward servo position.
     * Note: wheelServo position update is missing here, intentional?
     */
    @Override
    public void periodic() {
        upwardServo.setPosition(upwardServoPosition);
        
    }
}
