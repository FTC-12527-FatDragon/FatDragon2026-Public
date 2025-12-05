package org.firstinspires.ftc.teamcode.subsystems.wheel;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Wheel Subsystem
 *
 * This subsystem controls the "Wheel" mechanism, which includes a servo for rotating the wheel
 * and another servo for moving it up and down.
 * 
 * Now includes an internal State Machine for automated firing sequences.
 * AND Color Sensor integration for sample detection.
 */
public class Wheel extends SubsystemBase {

    public final Servo wheelServo;
    public final Servo upwardServo;
    public final ColorSensor colorSensor;
    public final DistanceSensor distanceSensor;

    public double upwardServoPosition = WheelConstants.upwardServoLow;
    public double customWheelPos = WheelConstants.posIDLE;

    public WheelServoState wheelState = WheelServoState.IDLE;
    public LauncherState launcherState = LauncherState.IDLE;

    public int currentSlotIndex = WheelConstants.INITIAL_SLOT_INDEX;
    
    // Flag to indicate direction of travel for slots
    // true = increasing index (forward), false = decreasing index (backward)
    private boolean isTraversingForward = true;
    
    private long stateTimer = 0;
    
    // Cache for color values
    private final float[] hsvValues = {0F, 0F, 0F};

    /**
     * Constructor for Wheel.
     * Initializes the servos and color sensor.
     *
     * @param hardwareMap The hardware map to get the devices.
     */
    public Wheel(HardwareMap hardwareMap) {
        wheelServo = hardwareMap.get(Servo.class, WheelConstants.wheelServoName);
        upwardServo = hardwareMap.get(Servo.class, WheelConstants.upwardServoName);
        
        // Initialize Color/Distance Sensor
        // Note: On Rev Color Sensor V3, these are usually the same device name
        colorSensor = hardwareMap.get(ColorSensor.class, WheelConstants.colorSensorName);
        distanceSensor = hardwareMap.get(DistanceSensor.class, WheelConstants.colorSensorName);
    }

    /**
     * Enum for the different states of the wheel servo.
     */
    public enum WheelServoState {
        IDLE(WheelConstants.posIDLE),
        ONE(WheelConstants.posOne),
        TWO(WheelConstants.posTwo),
        THREE (WheelConstants.posThree),
        CUSTOM(0);

        final double servoPos;

        WheelServoState(double servoPos){
            this.servoPos = servoPos;
        }
    }
    
    /**
     * Enum for the firing sequence state machine.
     */
    public enum LauncherState {
        IDLE,
        FIRING_UP,   // Servo moves UP
        FIRING_WAIT, // Waiting for shot to leave
        FIRING_DOWN  // Servo moves DOWN
    }
    
    public enum SampleColor {
        NONE, RED, BLUE, YELLOW
    }

    /**
     * Sets the state of the wheel servo.
     *
     * @param state The target state.
     */
    public void setWheelState(Wheel.WheelServoState state){
        wheelState = state;
    }

    public void setCustomWheelPos(double pos) {
        wheelState = WheelServoState.CUSTOM;
        customWheelPos = pos;
    }
    
    public void setUpwardServoPosition(double pos) {
        upwardServoPosition = pos;
    }

    /**
     * Toggles the upward servo position between two preset values.
     */
    public void toggleUpwardServo() {
        upwardServoPosition = upwardServoPosition == WheelConstants.upwardServoLow ? WheelConstants.upwardServoHigh : WheelConstants.upwardServoLow;
    }
    
    /**
     * Triggers the automated firing sequence.
     * This replaces the need for a complex LaunchSingleCommand.
     */
    public void fire() {
        if (launcherState == LauncherState.IDLE) {
            launcherState = LauncherState.FIRING_UP;
            stateTimer = System.currentTimeMillis();
        }
    }

    /**
     * Moves to the next slot.
     * Logic:
     * - If moving forward (isTraversingForward=true), increment index.
     * - If we reach the end (last index), switch direction to backward.
     * - If moving backward (isTraversingForward=false), decrement index.
     * - If we reach the start (index 0), switch direction to forward.
     */
    public void nextSlot() {
        if (isTraversingForward) {
            currentSlotIndex++;
            if (currentSlotIndex >= WheelConstants.WHEEL_SLOTS.length - 1) {
                // Reached the end, clamp to max and reverse direction
                currentSlotIndex = WheelConstants.WHEEL_SLOTS.length - 1;
                isTraversingForward = false;
            }
        } else {
            currentSlotIndex--;
            if (currentSlotIndex <= 0) {
                // Reached the start, clamp to 0 and reverse direction
                currentSlotIndex = 0;
                isTraversingForward = true;
            }
        }
        setCustomWheelPos(WheelConstants.WHEEL_SLOTS[currentSlotIndex]);
    }

    public void resetSlot() {
        currentSlotIndex = WheelConstants.INITIAL_SLOT_INDEX;
        // Reset direction logic too if needed? 
        // Assuming starting at 0 implies moving forward initially.
        isTraversingForward = true; 
        setCustomWheelPos(WheelConstants.WHEEL_SLOTS[currentSlotIndex]);
    }
    
    /**
     * Checks if a sample is currently detected in the wheel/magazine.
     * @return true if distance is less than threshold.
     */
    public boolean hasSample() {
        return distanceSensor.getDistance(DistanceUnit.CM) < WheelConstants.detectionThresholdCM;
    }
    
    /**
     * Gets the color of the detected sample.
     * Uses HSV Hue to distinguish colors.
     */
    public SampleColor getSampleColor() {
        if (!hasSample()) return SampleColor.NONE;
        
        // Convert RGB to HSV
        Color.RGBToHSV(
            (int) (colorSensor.red() * 255),
            (int) (colorSensor.green() * 255),
            (int) (colorSensor.blue() * 255),
            hsvValues
        );
        
        float hue = hsvValues[0];
        
        // Simple Hue Thresholding (Tune these values!)
        // Yellow: 60-100
        // Blue: 200-240
        // Red: 0-30 or 330-360
        
        if (hue > 200 && hue < 260) {
            return SampleColor.BLUE;
        } else if (hue > 45 && hue < 120) {
            return SampleColor.YELLOW;
        } else if (hue < 30 || hue > 330) {
            return SampleColor.RED;
        }
        
        return SampleColor.NONE; // Unknown color
    }

    /**
     * Periodic method called by the CommandScheduler.
     * Updates the upward servo position and handles the firing state machine.
     */
    @Override
    public void periodic() {
        // --- State Machine Logic ---
        switch (launcherState) {
            case IDLE:
                // Do nothing, just hold position
                break;
                
            case FIRING_UP:
                // Move servo UP
                upwardServoPosition = WheelConstants.upwardServoHigh;
                // Check if time passed (servo travel time + shot time)
                if (System.currentTimeMillis() - stateTimer > WheelConstants.launchWaitTime) {
                    launcherState = LauncherState.FIRING_DOWN;
                    stateTimer = System.currentTimeMillis();
                }
                break;
                
            case FIRING_DOWN:
                // Move servo DOWN
                upwardServoPosition = WheelConstants.upwardServoLow;
                // Sequence complete, go back to IDLE
                launcherState = LauncherState.IDLE;
                break;
                
            default:
                launcherState = LauncherState.IDLE;
                break;
        }
    
        // --- Hardware Write ---
        upwardServo.setPosition(upwardServoPosition);
        
        // Only allow wheel rotation if NOT firing to prevent jamming
        if (launcherState == LauncherState.IDLE) {
            if (wheelState == WheelServoState.CUSTOM) {
                wheelServo.setPosition(customWheelPos);
            } else {
                wheelServo.setPosition(wheelState.servoPos);
            }
        }
    }

    // --- Command Factories ---
    
    public Command fireCommand() {
        return new InstantCommand(this::fire, this);
    }

    public Command nextSlotCommand() {
        return new InstantCommand(this::nextSlot, this);
    }

    public Command resetSlotCommand() {
        return new InstantCommand(this::resetSlot, this);
    }
}
