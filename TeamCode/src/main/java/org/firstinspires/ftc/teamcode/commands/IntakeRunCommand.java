package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

/**
 * Command to run the intake.
 * Can specify direction (normal or reversed).
 * Stops the intake when the command ends.
 */
public class IntakeRunCommand extends CommandBase {
    private final Intake intake;
    private final boolean reversed;

    public IntakeRunCommand(Intake intake, boolean reversed) {
        this.intake = intake;
        this.reversed = reversed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.reverseMotor(reversed);
        intake.setRunning(true);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setRunning(false);
    }
}

