package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.gimbal.Gimbal;

/**
 * Manually adjusts the gimbal position while the command is running.
 */
public class GimbalManualControlCommand extends CommandBase {
    private final Gimbal gimbal;
    private final double adjustAmount;

    /**
     * @param gimbal The gimbal subsystem.
     * @param adjustAmount The amount to adjust per loop (e.g. 0.001 or -0.001).
     */
    public GimbalManualControlCommand(Gimbal gimbal, double adjustAmount) {
        this.gimbal = gimbal;
        this.adjustAmount = adjustAmount;
        addRequirements(gimbal);
    }

    @Override
    public void initialize() {
        gimbal.setGimbalState(Gimbal.GimbalServoState.AIM);
    }

    @Override
    public void execute() {
        Gimbal.gimbalServoPosition += adjustAmount;
        // Optional: Clamp values to 0.0 - 1.0
        if (Gimbal.gimbalServoPosition > 1.0) Gimbal.gimbalServoPosition = 1.0;
        if (Gimbal.gimbalServoPosition < 0.0) Gimbal.gimbalServoPosition = 0.0;
    }

    @Override
    public boolean isFinished() {
        return false; // Runs while button is held
    }
}

