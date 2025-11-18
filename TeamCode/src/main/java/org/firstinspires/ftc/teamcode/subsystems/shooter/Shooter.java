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
    public final DcMotorRe shooterUp, shooterDown;
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
        shooterUp = new DcMotorRe(hardwareMap, ShooterConstants.upShooterName);
        shooterDown = new DcMotorRe(hardwareMap, ShooterConstants.downShooterName);
        pidController = new PIDController(ShooterConstants.kP,
                ShooterConstants.kI, ShooterConstants.kD);
    }

    public double getAverageVelocity() {
        return (shooterUp.getAverageVelocity()+shooterDown.getAverageVelocity())/2;
    }

    public double getInstantVelocityUp() {
        return shooterUp.getInstantVelocity();
    }

    public double getInstantVelocityDown() {
        return shooterDown.getInstantVelocity();
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
                shooterUp.getLibVelocity(), ShooterConstants.shooterEpsilon);
    }

    @Override
    public void periodic() {
        if (shooterState != ShooterState.OPENLOOP) {
            if (shooterState != ShooterState.STOP) shooterUp.setPower(pidController.calculate(
                    shooterUp.getLibVelocity(), shooterState.shooterVelocity));
            else shooterUp.setPower(ShooterState.STOP.shooterVelocity);
        }
        else shooterUp.setPower(shooterOpenLoopPower);
        shooterUp.updateLastPos();
    }
}
