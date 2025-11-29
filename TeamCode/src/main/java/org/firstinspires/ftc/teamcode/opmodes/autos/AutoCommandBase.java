package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.wheel.Wheel;

/**
 * Base class for all autonomous OpModes.
 * Handles initialization of subsystems and the CommandScheduler loop.
 */
public abstract class AutoCommandBase extends LinearOpMode {
    protected Shooter shooter;
    protected Intake intake;
    protected Wheel wheel;
    protected Follower follower;
    protected Telemetry telemetryM;

    /**
     * Must be implemented by the specific Auto class to return the main command sequence.
     */
    public abstract Command runAutoCommand();

    /**
     * Must be implemented to return the starting pose of the robot.
     */
    public abstract Pose getStartPose();

    private void initialize() {
        // Initialize Telemetry
        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Pedro Pathing Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(getStartPose());

        // Initialize Subsystems
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        wheel = new Wheel(hardwareMap);

        // Set initial states if needed
        shooter.setShooterState(Shooter.ShooterState.STOP);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        // Build the command sequence
        Command autoCommand = runAutoCommand();

        // Schedule the command
        CommandScheduler.getInstance().schedule(autoCommand);

        waitForStart();

        if (isStopRequested()) return;

        // Main Loop
        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            follower.update(); // Important for Pedro Pathing
            
            // Telemetry
            telemetryM.addData("X", follower.getPose().getX());
            telemetryM.addData("Y", follower.getPose().getY());
            telemetryM.addData("Heading", follower.getPose().getHeading());
            telemetryM.update();
        }

        CommandScheduler.getInstance().reset();
    }
}

