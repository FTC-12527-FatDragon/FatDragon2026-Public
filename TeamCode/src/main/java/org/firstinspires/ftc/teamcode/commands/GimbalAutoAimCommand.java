package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.drive.AutoPaths;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.gimbal.Gimbal;
import org.firstinspires.ftc.teamcode.utils.AimCalculator;

/**
 * Continuously aims the gimbal at the Blue Basket while the command is running.
 */
public class GimbalAutoAimCommand extends CommandBase {
    private final Gimbal gimbal;
    private final MecanumDrive drive;
    private final Pose targetPose;

    public GimbalAutoAimCommand(Gimbal gimbal, MecanumDrive drive) {
        this.gimbal = gimbal;
        this.drive = drive;
        this.targetPose = AutoPaths.BLUE_BASKET; // Default target, can be parameterized if needed
        addRequirements(gimbal);
    }

    @Override
    public void initialize() {
        gimbal.setGimbalState(Gimbal.GimbalServoState.AIM);
    }

    @Override
    public void execute() {
        // Get robot pose
        Pose robotPose = drive.getPose();
        
        // Calculate aim position
        double aimPos = AimCalculator.getAutoAimServoPos(robotPose, targetPose);
        
        // Update gimbal position
        Gimbal.gimbalServoPosition = aimPos;
        
        // Note: Gimbal.periodic() will automatically apply this position because state is AIM
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted (button released)
    }
}

