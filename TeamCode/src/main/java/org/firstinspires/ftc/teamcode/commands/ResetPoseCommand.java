package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;

/**
 * Command to completely reset the robot's pose to (0,0,0).
 * Useful for emergency recalibration.
 */
public class ResetPoseCommand extends InstantCommand {
    public ResetPoseCommand(MecanumDrive drive) {
        super(() -> drive.follower.setPose(new Pose(0, 0, 0)));
    }
}

