package org.firstinspires.ftc.teamcode.subsystems.wheel;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Wheel Subsystem
 *
 * This subsystem controls the "Wheel" mechanism, which includes a servo for rotating the wheel
 * and another servo for moving it up and down.
 * 
 * Now includes an internal State Machine for automated firing sequences.
 */
public class Wheel extends SubsystemBase {

    public final Servo wheelServo;
    public final Servo upwardServo;

    public double upwardServoPosition = WheelConstants.upwardServoLow;
    public double customWheelPos = WheelConstants.posIDLE;

    public WheelServoState wheelState = WheelServoState.IDLE;
    public LauncherState launcherState = LauncherState.IDLE;

    public int currentSlotIndex = WheelConstants.INITIAL_SLOT_INDEX;
    
    private long stateTimer = 0;

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

    public void nextSlot() {
        currentSlotIndex++;
        if (currentSlotIndex >= WheelConstants.WHEEL_SLOTS.length) {
            currentSlotIndex = 0; 
        }
        setCustomWheelPos(WheelConstants.WHEEL_SLOTS[currentSlotIndex]);
    }

    public void resetSlot() {
        currentSlotIndex = WheelConstants.INITIAL_SLOT_INDEX;
        setCustomWheelPos(WheelConstants.WHEEL_SLOTS[currentSlotIndex]);
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
        
        if (wheelState == WheelServoState.CUSTOM) {
            wheelServo.setPosition(customWheelPos);
        } else {
            wheelServo.setPosition(wheelState.servoPos);
        }
    }
}
