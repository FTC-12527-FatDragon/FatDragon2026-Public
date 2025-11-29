package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

/**
 * A Command wrapper for Pedro Pathing's Follower.
 * This allows us to use Pedro Pathing paths within a Command-Based structure.
 */
public class AutoDriveCommand extends CommandBase {
    private final Follower follower;
    private final PathChain pathChain;
    private final boolean holdEnd;

    public AutoDriveCommand(Follower follower, PathChain pathChain) {
        this(follower, pathChain, true);
    }

    public AutoDriveCommand(Follower follower, PathChain pathChain, boolean holdEnd) {
        this.follower = follower;
        this.pathChain = pathChain;
        this.holdEnd = holdEnd;
    }

    @Override
    public void initialize() {
        follower.followPath(pathChain, holdEnd);
    }

    @Override
    public void execute() {
        follower.update();
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            follower.breakFollowing();
        }
    }
}

