package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Intake Subsystem
 *
 * This subsystem controls the intake mechanism, which brings game elements into the robot.
 * It consists of a motor that can be toggled on/off and reversed.
 * Includes current monitoring to detect stalls.
 */
public class Intake extends SubsystemBase {
    public final DcMotorEx intakeMotor;

    public boolean isRunning, motorReversed;

    /**
     * Constructor for Intake.
     * Initializes the intake motor.
     *
     * @param hardwareMap The hardware map to get the motor.
     */
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, IntakeConstants.intakeMotorName);
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
     * Gets the current draw of the intake motor.
     * @return Current in Amps.
     */
    public double getCurrent() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
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
            
            // Check for stall
            double current = getCurrent();
            if (current > IntakeConstants.stallCurrentThreshold) {
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("WARNING", "INTAKE STALL DETECTED!");
                packet.put("Intake Current", current);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

        }
        else {
            intakeMotor.setPower(0);
        }
    }
}
