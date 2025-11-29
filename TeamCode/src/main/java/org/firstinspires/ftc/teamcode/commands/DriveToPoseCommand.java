package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;

/**
 * Command to drive the robot to a specific pose using Pedro Pathing.
 */
public class DriveToPoseCommand extends CommandBase {
    private final MecanumDrive drive;
    private final Pose targetPose;
    private PathChain pathChain;

    public DriveToPoseCommand(MecanumDrive drive, Pose targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Pose startPose = drive.getPose();
        
        // Build a path from current pose to target pose
        // We use a simple BezierLine for point-to-point movement
        pathChain = drive.follower.pathBuilder()
                .addPath(new BezierLine(startPose, targetPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
                .build();
                
        drive.follower.followPath(pathChain, true); // true = holdEnd
    }

    @Override
    public void execute() {
        drive.follower.update();
    }

    @Override
    public boolean isFinished() {
        // Command finishes when the follower is no longer busy (path completed)
        return !drive.follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        // If interrupted (e.g. by stick movement in TeleOp), stop following
        if (interrupted) {
            drive.follower.breakFollowing();
        }
        // Return to TeleOp drive control
        drive.follower.startTeleopDrive();
    }
}

