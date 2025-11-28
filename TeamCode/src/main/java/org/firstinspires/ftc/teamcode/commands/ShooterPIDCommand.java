package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/**
 * Command to run the shooter using PID closed-loop control.
 * Sets the shooter state to the target state (SLOW/FAST) on initialize,
 * and stops it on end.
 */
public class ShooterPIDCommand extends CommandBase {
    private final Shooter shooter;
    private final Shooter.ShooterState targetState;

    public ShooterPIDCommand(Shooter shooter, Shooter.ShooterState targetState) {
        this.shooter = shooter;
        this.targetState = targetState;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterState(targetState);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterState(Shooter.ShooterState.STOP);
    }
}

