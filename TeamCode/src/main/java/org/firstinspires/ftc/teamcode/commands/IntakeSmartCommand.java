package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.wheel.Wheel;

/**
 * Runs the intake only if the wheel magazine is not full.
 * If a sample is detected in the wheel, the intake stops automatically.
 */
public class IntakeSmartCommand extends CommandBase {
    private final Intake intake;
    private final Wheel wheel;
    private final boolean reverse;

    public IntakeSmartCommand(Intake intake, Wheel wheel, boolean reverse) {
        this.intake = intake;
        this.wheel = wheel;
        this.reverse = reverse;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (reverse) {
            // Always allow reversing (outtaking) even if full
            intake.intakeMotor.setPower(IntakeConstants.reversedPower);
            intake.setRunning(true);
            intake.reverseMotor(true);
        } else {
            // Intaking: Check if full
            if (!wheel.hasSample()) {
                intake.intakeMotor.setPower(IntakeConstants.intakePower);
                intake.setRunning(true);
                intake.reverseMotor(false);
            } else {
                // Full! Stop.
                intake.intakeMotor.setPower(0);
                intake.setRunning(false);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.intakeMotor.setPower(0);
        intake.setRunning(false);
    }
}

