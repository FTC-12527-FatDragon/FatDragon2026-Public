package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.DcMotorRe;
import org.firstinspires.ftc.teamcode.utils.Util;

public class Shooter extends SubsystemBase {
    public final DcMotorRe upShooter, downShooter;
    public final PIDController pidController;
    public static double shooterOpenLoopPower = -1;

    public enum ShooterState {
        STOP(ShooterConstants.stopVelocity),
        SLOW(ShooterConstants.slowVelocity),
        FAST(ShooterConstants.fastVelocity),
        OPENLOOP(0);

        final double shooterVelocity;

        ShooterState(double shooterVelocity) {
            this.shooterVelocity = shooterVelocity;
        }
    }

    public ShooterState shooterState = ShooterState.STOP;

    public Shooter(final HardwareMap hardwareMap) {
        upShooter = new DcMotorRe(hardwareMap, ShooterConstants.upShooterName);
        downShooter = new DcMotorRe(hardwareMap, ShooterConstants.downShooterName);
        pidController = new PIDController(ShooterConstants.kP,
                ShooterConstants.kI, ShooterConstants.kD);
    }

    public double getAverageVelocity() {
        return (upShooter.getAverageVelocity()+downShooter.getAverageVelocity())/2;
    }

    public double getInstantVelocityUp() {
        return upShooter.getInstantVelocity();
    }

    public double getInstantVelocityDown() {
        return downShooter.getInstantVelocity();
    }

    public void toggleShooterState(ShooterState shooterStateE) {
        if (shooterStateE == ShooterState.SLOW) {
            shooterState = shooterState == ShooterState.STOP ? ShooterState.SLOW : ShooterState.STOP;
        }
        else if (shooterStateE == ShooterState.FAST) {
            shooterState = shooterState == ShooterState.STOP ? ShooterState.FAST : ShooterState.STOP;
        }
    }

    public void setOpenLoopPower(double power) {
        shooterState = ShooterState.OPENLOOP;
        shooterOpenLoopPower = power;
    }

    public void setShooterState(ShooterState state) {
        shooterState = state;
    }

    public boolean isShooterAtSetPoint() {
        return Util.epsilonEqual(shooterState.shooterVelocity,
                upShooter.getLibVelocity(), ShooterConstants.shooterEpsilon);
    }

    @Override
    public void periodic() {
        if (shooterState != ShooterState.OPENLOOP) {
            if (shooterState != ShooterState.STOP) {
                double currentPower = pidController.calculate(
                        upShooter.getLibVelocity(), shooterState.shooterVelocity);
                upShooter.setPower(currentPower);
                downShooter.setPower(-currentPower);
            }
            else {
                upShooter.setPower(ShooterState.STOP.shooterVelocity);
                downShooter.setPower(-ShooterState.STOP.shooterVelocity);
            }
        }
        else {
            upShooter.setPower(shooterOpenLoopPower);
            downShooter.setPower(-shooterOpenLoopPower);
        }
        upShooter.updateLastPos();
    }
}
