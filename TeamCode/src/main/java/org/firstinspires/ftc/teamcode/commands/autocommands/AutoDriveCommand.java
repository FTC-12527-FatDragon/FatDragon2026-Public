package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

/**
 * A Command to execute a Pedro Pathing PathChain.
 * Can be used in both Auto and TeleOp (for semi-auto navigation).
 */
public class AutoDriveCommand extends CommandBase {
    private final Follower follower;
    private final PathChain pathChain;
    private final boolean holdEnd;

    /**
     * Constructor.
     * @param follower The Pedro Pathing follower instance.
     * @param pathChain The path to follow.
     */
    public AutoDriveCommand(Follower follower, PathChain pathChain) {
        this(follower, pathChain, true);
    }

    /**
     * Constructor with holdEnd option.
     * @param follower The Pedro Pathing follower instance.
     * @param pathChain The path to follow.
     * @param holdEnd Whether to hold the position at the end of the path.
     */
    public AutoDriveCommand(Follower follower, PathChain pathChain, boolean holdEnd) {
        this.follower = follower;
        this.pathChain = pathChain;
        this.holdEnd = holdEnd;
        // Note: We generally don't addRequirements(drive) here because the Follower handles the drive hardware internally,
        // and we might want to run this in parallel with other subsystems. 
        // If you want exclusive control, you'd need to wrap Follower in a Subsystem.
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
        // The command finishes when the follower is no longer busy (path completed).
        return !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            follower.breakFollowing();
        }
    }
}
