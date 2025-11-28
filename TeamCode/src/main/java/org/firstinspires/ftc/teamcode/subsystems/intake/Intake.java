package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;

/**
 * Intake Subsystem
 *
 * This subsystem controls the intake mechanism, which brings game elements into the robot.
 * It consists of a motor that can be toggled on/off and reversed.
 */
public class Intake extends SubsystemBase {
    public final DcMotor intakeMotor;

    public static boolean isRunning, motorReversed;

    /**
     * Constructor for Intake.
     * Initializes the intake motor.
     *
     * @param hardwareMap The hardware map to get the motor.
     */
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, IntakeConstants.intakeMotorName);


        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        isRunning = false;
    }

    /**
     * Toggles the intake running state (On/Off).
     */
    public void toggle() {
        isRunning = !isRunning;
    }

    /**
     * Sets the intake running state directly.
     *
     * @param running True to run, false to stop.
     */
    public void setRunning(boolean running) {
        isRunning = running;
    }

    /**
     * Sets the motor direction (Normal/Reversed).
     *
     * @param inverse True for reversed (outtake), false for normal (intake).
     */
    public void reverseMotor(boolean inverse) { motorReversed = inverse; }

    /**
     * Checks if the intake is currently running.
     *
     * @return True if running, false otherwise.
     */
    public boolean isRunning() {
        return isRunning;
    }

    /**
     * Periodic method called by the CommandScheduler.
     * Updates the intake motor power based on the current state.
     */
    @Override
    public void periodic() {
        if (isRunning) {
            if (motorReversed) {
                intakeMotor.setPower(IntakeConstants.reversedPower);
            } else {
                intakeMotor.setPower(IntakeConstants.intakePower);
            }

        }
        else {
            intakeMotor.setPower(0);

        }


    }
}
