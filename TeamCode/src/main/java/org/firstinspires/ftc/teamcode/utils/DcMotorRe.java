package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

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
    private final double WINDOW = 10;
    // Stores historical velocities for averaging
    private final Deque<Double> velocityList = new ArrayDeque<>();

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
     * Gets the velocity directly from the motor controller (built-in method).
     * This is more accurate than calculating it manually in Java due to loop timing jitter.
     *
     * @return Velocity in ticks per second.
     */
    public double getInstantVelocity() {
        return motor.getVelocity();
    }

    /**
     * Calculates the average velocity over the sliding window.
     *
     * @return Average velocity in ticks per second.
     */
    public double getAverageVelocity() {
        if (velocityList.isEmpty()) return 0;
        
        double sum = 0;
        for (double v : velocityList) {
            sum += v;
        }
        return sum / velocityList.size();
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
     * Gets the current draw of the motor.
     *
     * @param unit The unit of current (e.g. AMPS, MILLIAMPS).
     * @return Current in the specified unit.
     */
    public double getCurrent(CurrentUnit unit) {
        return motor.getCurrent(unit);
    }

    /**
     * Updates the sliding window of velocities.
     * This should be called once per loop cycle.
     */
    public void updateLastPos() {
        if (velocityList.size() >= WINDOW) {
            velocityList.removeFirst();
        }
        velocityList.addLast(motor.getVelocity());
    }
}
