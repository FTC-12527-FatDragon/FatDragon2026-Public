package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayDeque;
import java.util.Deque;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * DcMotorRe (DcMotor Re-implemented/Extended)
 *
 * This is a wrapper class for DcMotorEx that provides additional functionality,
 * specifically for calculating velocity using a custom moving average window.
 * This can be useful for getting smoother velocity readings than the built-in .getVelocity().
 */
public class DcMotorRe {
    private final DcMotorEx motor;
    private double lastPos = 0;
    private final double WINDOW = 10;
    private final Deque<Double> posList = new ArrayDeque<>();

    /**
     * Constructor for DcMotorRe.
     *
     * @param hardwareMap The hardware map to get the motor.
     * @param motorName   The name of the motor in the hardware map.
     */
    public DcMotorRe(final HardwareMap hardwareMap, final String motorName) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets the direction of the motor.
     *
     * @param direction The direction (FORWARD or REVERSE).
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    /**
     * Gets the current position of the motor in ticks.
     *
     * @return Current position.
     */
    public double getPosition() {
        return motor.getCurrentPosition();
    }

    /**
     * Calculates the instantaneous velocity based on the difference between the current
     * and last recorded position. Assumes a loop time of 20ms (0.02s).
     *
     * @return Instantaneous velocity in ticks per second.
     */
    public double getInstantVelocity() {
        return (motor.getCurrentPosition() - lastPos) / 0.02;
    }

    /**
     * Calculates the average velocity over the sliding window of recorded positions.
     * Assumes a loop time of 20ms (0.02s).
     *
     * @return Average velocity in ticks per second.
     */
    public double getAverageVelocity() {
        if (posList.peekFirst() != null && posList.peekLast() != null)
            return (posList.peekLast() - posList.peekFirst()) / (WINDOW * 0.02);
        return 0;
    }

    /**
     * Sets the power of the motor.
     *
     * @param power The power to set [-1.0, 1.0].
     */
    public void setPower(double power) {
        motor.setPower(power);
    }

    /**
     * Gets the velocity directly from the motor controller (built-in method).
     *
     * @return Velocity in ticks per second.
     */
    public double getLibVelocity() {
        return motor.getVelocity();
    }

    /**
     * Updates the last recorded position and the sliding window of positions.
     * This should be called once per loop cycle.
     */
    public void updateLastPos() {
        lastPos = motor.getCurrentPosition();
        if (posList.size() >= WINDOW) {
            posList.removeFirst();
        }
        posList.addLast(lastPos);
    }
}
