package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;

public class Intake extends SubsystemBase {
    public final DcMotor intakeMotor;



    public static boolean isRunning, motorReversed;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, IntakeConstants.intakeMotorName);


        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        isRunning = false;
    }

    public void toggle() {
        isRunning = !isRunning;
    }

    public void reverseMotor(boolean inverse) { motorReversed = inverse; }

    public boolean isRunning() {
        return isRunning;
    }

    @Override
    public void periodic() {
        if (isRunning) {
            if (motorReversed) {
                intakeMotor.setPower(IntakeConstants.reversedPower);
            } else {
                intakeMotor.setPower(IntakeConstants.intakePower);
            }

        }
        else {
            intakeMotor.setPower(0);

        }


    }
}
