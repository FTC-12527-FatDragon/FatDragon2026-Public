package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/**
 * Command to manually run the shooter at a fixed power.
 * Stops the shooter when the command ends (e.g., button released).
 */
public class ShooterManualCommand extends CommandBase {
    private final Shooter shooter;
    private final double power;

    public ShooterManualCommand(Shooter shooter, double power) {
        this.shooter = shooter;
        this.power = power;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setOpenLoopPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setOpenLoopPower(0);
    }
}

