package org.firstinspires.ftc.teamcode.subsystems.gimbal;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.arcrobotics.ftclib.command.Command;
import org.firstinspires.ftc.teamcode.commands.GimbalAutoAimCommand;
import org.firstinspires.ftc.teamcode.commands.GimbalManualControlCommand;
import org.firstinspires.ftc.teamcode.commands.GimbalResetCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;

/**
 * Gimbal Subsystem
 *
 * This subsystem controls a single servo gimbal mechanism used to aim or position elements.
 * It supports preset positions (IDLE, 90, 180) and a custom AIM position.
 */
public class Gimbal extends SubsystemBase {

    public final Servo gimbalServo;
    // Initialize with the Constant value
    private static double gimbalServoPosition = GimbalConstants.posIDLE;

    public GimbalServoState gimbalState = GimbalServoState.IDLE;

    /**
     * Sets the target position for the gimbal servo (0.0 - 1.0).
     * @param pos Target position.
     */
    public static void setTargetPosition(double pos) {
        gimbalServoPosition = pos;
    }

    /**
     * Gets the current target position of the gimbal servo.
     * @return Current target position (0.0 - 1.0).
     */
    public static double getTargetPosition() {
        return gimbalServoPosition;
    }

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

    // --- Command Factories ---

    public Command autoAimCommand(MecanumDrive drive) {
        return new GimbalAutoAimCommand(this, drive);
    }

    public Command manualControlCommand(double adjustAmount) {
        return new GimbalManualControlCommand(this, adjustAmount);
    }

    public Command resetCommand() {
        return new GimbalResetCommand(this);
    }
}
