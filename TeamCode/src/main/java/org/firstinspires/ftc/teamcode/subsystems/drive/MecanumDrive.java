package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class MecanumDrive extends SubsystemBase {
    public final Follower follower;

    public MecanumDrive(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        
        // Set all drive motors to BRAKE mode for active stopping
        setMotorZeroPowerBehavior(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Load the saved pose from Auto if available. 
        // If PoseStorage.currentPose is (0,0,0) it might mean Auto didn't run, 
        // but since we initialize it to (0,0,0) anyway, it's fine.
        follower.setStartingPose(PoseStorage.currentPose);
        
        follower.startTeleopDrive();
    }
    
    /**
     * Helper method to set the ZeroPowerBehavior for all drive motors.
     */
    private void setMotorZeroPowerBehavior(HardwareMap hardwareMap, DcMotor.ZeroPowerBehavior behavior) {
        try {
            hardwareMap.get(DcMotor.class, DriveConstants.leftFrontMotorName).setZeroPowerBehavior(behavior);
            hardwareMap.get(DcMotor.class, DriveConstants.leftBackMotorName).setZeroPowerBehavior(behavior);
            hardwareMap.get(DcMotor.class, DriveConstants.rightFrontMotorName).setZeroPowerBehavior(behavior);
            hardwareMap.get(DcMotor.class, DriveConstants.rightBackMotorName).setZeroPowerBehavior(behavior);
        } catch (Exception e) {
            // Log error or ignore if testing without full hardware
        }
    }

    @Override
    public void periodic() {
        follower.update();
    }

    public void moveRobotFieldRelative(double forward, double strafe, double turn) {
        // Inputs are typically from gamepad: 
        // forward (Y stick): Up is -1
        // strafe (X stick): Right is +1
        // turn (RX stick): Right is +1
        
        // Pedro expects:
        // xVelocity: + is Forward
        // yVelocity: + is Left
        // headingVelocity: + is CCW (Left)
        
        // Conversions:
        // Up (-1) -> Forward (+1) => -forward
        // Right (+1) -> Right (-1) => -strafe
        // Right (+1) -> CW (-1) => -turn
        
        follower.setTeleOpDrive(-forward, -strafe, -turn, true);
    }

    public void moveRobot(double forward, double strafe, double turn) {
        follower.setTeleOpDrive(-forward, -strafe, -turn, false);
    }
    
    public void reset(double heading) {
        Pose current = follower.getPose();
        follower.setPose(new Pose(current.getX(), current.getY(), heading));
    }

    public Pose getPose() {
        return follower.getPose();
    }
    
    public double getYawOffset() {
        // Not strictly tracked separately anymore, but we can imply it 
        // if we want to maintain API compatibility. 
        // But for now, just return 0 or current heading.
        return 0;
    }
}
