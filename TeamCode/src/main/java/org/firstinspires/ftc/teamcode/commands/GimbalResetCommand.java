package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.gimbal.Gimbal;
import org.firstinspires.ftc.teamcode.subsystems.gimbal.GimbalConstants;

/**
 * Resets the gimbal to the IDLE (Center) position.
 */
public class GimbalResetCommand extends InstantCommand {
    public GimbalResetCommand(Gimbal gimbal) {
        super(
            () -> {
                Gimbal.gimbalServoPosition = GimbalConstants.posIDLE;
                gimbal.setGimbalState(Gimbal.GimbalServoState.AIM);
            },
            gimbal
        );
    }
}

