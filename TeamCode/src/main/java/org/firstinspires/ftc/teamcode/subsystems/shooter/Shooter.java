package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.DcMotorRe;
import org.firstinspires.ftc.teamcode.utils.Util;

/**
 * Shooter Subsystem
 *
 * This subsystem controls the robot's shooter mechanism, consisting of two flywheels (up and down).
 * It uses PID control to maintain target velocities for consistent shooting.
 */
public class Shooter extends SubsystemBase {
    public final DcMotorRe shooterUp, shooterDown;
    public final PIDController pidControllerUp, pidControllerDown;
    public static double shooterOpenLoopPower = -1;

    /**
     * Enum representing the different states of the shooter.
     */
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

    /**
     * Constructor for Shooter.
     * Initializes motors and PID controllers.
     *
     * @param hardwareMap The hardware map to get the motors.
     */
    public Shooter(final HardwareMap hardwareMap) {
        shooterUp = new DcMotorRe(hardwareMap, ShooterConstants.upShooterName);
        shooterDown = new DcMotorRe(hardwareMap, ShooterConstants.downShooterName);
        shooterDown.setDirection(DcMotorSimple.Direction.REVERSE);
        
        pidControllerUp = new PIDController(ShooterConstants.kP,
                ShooterConstants.kI, ShooterConstants.kD);
        pidControllerDown = new PIDController(ShooterConstants.kP,
                ShooterConstants.kI, ShooterConstants.kD);
    }

    /**
     * Gets the average velocity of both shooter motors.
     *
     * @return Average velocity in ticks per second.
     */
    public double getAverageVelocity() {
        return (shooterUp.getAverageVelocity()+shooterDown.getAverageVelocity())/2;
    }

    public double getInstantVelocityUp() {
        return shooterUp.getInstantVelocity();
    }

    public double getInstantVelocityDown() {
        return shooterDown.getInstantVelocity();
    }

    /**
     * Toggles the shooter state between STOP and the given state.
     * Note: This logic might be confusing. If the current state is ANY of the active states (SLOW or FAST),
     * toggling with a different active state argument will stop it first?
     * Current logic: 
     * If arg is SLOW: toggle between STOP and SLOW.
     * If arg is FAST: toggle between STOP and FAST.
     * 
     * @param shooterStateE The target state to toggle to.
     */
    public void toggleShooterState(ShooterState shooterStateE) {
        if (shooterStateE == ShooterState.SLOW) {
            shooterState = shooterState == ShooterState.STOP ? ShooterState.SLOW : ShooterState.STOP;
        }
        else if (shooterStateE == ShooterState.FAST) {
            shooterState = shooterState == ShooterState.STOP ? ShooterState.FAST : ShooterState.STOP;
        }
    }

    /**
     * Sets the shooter power directly (Open Loop Control).
     *
     * @param power The power to set [-1.0, 1.0].
     */
    public void setOpenLoopPower(double power) {
        shooterState = ShooterState.OPENLOOP;
        shooterOpenLoopPower = power;
    }

    /**
     * Sets the shooter state.
     *
     * @param state The target state.
     */
    public void setShooterState(ShooterState state) {
        shooterState = state;
    }

    /**
     * Checks if the shooter is at the target velocity.
     *
     * @return True if within tolerance, false otherwise.
     */
    public boolean isShooterAtSetPoint() {
        return Util.epsilonEqual(shooterState.shooterVelocity,
                shooterUp.getLibVelocity(), ShooterConstants.shooterEpsilon) &&
               Util.epsilonEqual(shooterState.shooterVelocity,
                shooterDown.getLibVelocity(), ShooterConstants.shooterEpsilon);
    }

    /**
     * Periodic method called by the CommandScheduler.
     * Updates the motor powers based on the current state and PID controller.
     */
    @Override
    public void periodic() {
        if (shooterState != ShooterState.OPENLOOP) {
            if (shooterState != ShooterState.STOP) {
                double targetVel = shooterState.shooterVelocity;
                
                // PID Calculation
                double pidUp = pidControllerUp.calculate(shooterUp.getLibVelocity(), targetVel);
                double pidDown = pidControllerDown.calculate(shooterDown.getLibVelocity(), targetVel);
                
                // Feedforward Calculation: FF = kV * targetVel + kS * sign(targetVel)
                double ff = (targetVel * ShooterConstants.kV) + (Math.signum(targetVel) * ShooterConstants.kS);
                
                // Total Power
                double powerUp = pidUp + ff;
                double powerDown = pidDown + ff;
                
                shooterUp.setPower(powerUp);
                shooterDown.setPower(powerDown);
            } else {
                shooterUp.setPower(ShooterState.STOP.shooterVelocity);
                shooterDown.setPower(ShooterState.STOP.shooterVelocity);
            }
        }
        else {
            shooterUp.setPower(shooterOpenLoopPower);
            shooterDown.setPower(shooterOpenLoopPower);
        }
        shooterUp.updateLastPos();
        shooterDown.updateLastPos();
    }
}
