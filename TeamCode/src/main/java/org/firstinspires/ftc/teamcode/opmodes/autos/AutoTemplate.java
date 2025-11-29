package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

@Config
@Autonomous(name = "Auto Template", group = "Templates")
public class AutoTemplate extends AutoCommandBase {

    // Define PathChains
    private PathChain path1;
    private PathChain path2;

    @Override
    public Pose getStartPose() {
        // Return the starting pose (x, y, heading in radians)
        return new Pose(0, 0, Math.toRadians(0));
    }

    @Override
    public Command runAutoCommand() {
        buildPaths();

        // Define the sequence of commands
        return new SequentialCommandGroup(
                // Example: Start Shooter
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST)),
                
                // Drive Path 1
                new AutoDriveCommand(follower, path1),
                
                // Wait for 1 second
                new WaitCommand(1000),
                
                // Drive Path 2
                new AutoDriveCommand(follower, path2),
                
                // Stop Shooter
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }

    private void buildPaths() {
        // Example Path 1: Move forward 24 inches
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(0, 0, 0),
                        new Pose(24, 0, 0)
                ))
                .setConstantHeadingInterpolation(0)
                .build();

        // Example Path 2: Move back to start
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(24, 0, 0),
                        new Pose(0, 0, 0)
                ))
                .setConstantHeadingInterpolation(0)
                .build();
    }
}

