package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.wheel.WheelConstants;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

@Config
@TeleOp(name = "ShooterTestOpMode", group = "Test")
public class ShooterTestOpMode extends CommandOpMode {

    private Servo wheelServo;
    private Servo upwardServo;
    private DcMotorEx shooterUp, shooterDown;
    private GamepadEx gamepadEx;

    // 定义目标位置常量 (步进 0.074)
    public static double WHEEL_POS_INIT = 0.832;
    public static double WHEEL_POS_2 = 0.758;
    public static double WHEEL_POS_3 = 0.684;
    public static double WHEEL_POS_4 = 0.610;
    public static double WHEEL_POS_5 = 0.536;
    public static double WHEEL_POS_6 = 0.462;

    public static double UPWARD_POS_HIGH = 0.6;
    public static double UPWARD_POS_LOW = 0.2;

    public static long SERVO_WAIT_TIME = 500; // 动作切换等待
    public static long LUNCH_WAIT_TIME = 0; // 在上面停留时间
    public static double SHOOTER_POWER = 1.0; 

    @Override
    public void initialize() {
        // 直接获取硬件，绕过 Subsystem 的 periodic 逻辑
        wheelServo = hardwareMap.get(Servo.class, WheelConstants.wheelServoName);
        upwardServo = hardwareMap.get(Servo.class, WheelConstants.upwardServoName);
        
        shooterUp = hardwareMap.get(DcMotorEx.class, ShooterConstants.upShooterName);
        shooterDown = hardwareMap.get(DcMotorEx.class, ShooterConstants.downShooterName);

        // Shooter 配置
        shooterDown.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        gamepadEx = new GamepadEx(gamepad1);

        schedule(new InstantCommand(() -> {
            shooterUp.setPower(-SHOOTER_POWER);
            shooterDown.setPower(-SHOOTER_POWER);
            
            wheelServo.setPosition(WHEEL_POS_INIT);
            upwardServo.setPosition(UPWARD_POS_LOW);
        }));

        // 定义 A 键序列
        new FunctionalButton(
                () -> gamepadEx.getButton(GamepadKeys.Button.A)
        ).whenPressed(
                new SequentialCommandGroup(
                        // 1. Lunch at POS_INIT
                        new InstantCommand(() -> upwardServo.setPosition(UPWARD_POS_HIGH)),
                        new WaitCommand(LUNCH_WAIT_TIME),
                        new InstantCommand(() -> upwardServo.setPosition(UPWARD_POS_LOW)),
                        new WaitCommand(SERVO_WAIT_TIME),

                        // Move to POS_2
                        new InstantCommand(() -> wheelServo.setPosition(WHEEL_POS_2)),
                        new WaitCommand(SERVO_WAIT_TIME),

                        // 2. Lunch at POS_2
                        new InstantCommand(() -> upwardServo.setPosition(UPWARD_POS_HIGH)),
                        new WaitCommand(LUNCH_WAIT_TIME),
                        new InstantCommand(() -> upwardServo.setPosition(UPWARD_POS_LOW)),
                        new WaitCommand(SERVO_WAIT_TIME),

                        // Move to POS_3
                        new InstantCommand(() -> wheelServo.setPosition(WHEEL_POS_3)),
                        new WaitCommand(SERVO_WAIT_TIME),

                        // 3. Lunch at POS_3
                        new InstantCommand(() -> upwardServo.setPosition(UPWARD_POS_HIGH)),
                        new WaitCommand(LUNCH_WAIT_TIME),
                        new InstantCommand(() -> upwardServo.setPosition(UPWARD_POS_LOW)),
                        new WaitCommand(SERVO_WAIT_TIME),
                        
                        // Move to POS_4
                        new InstantCommand(() -> wheelServo.setPosition(WHEEL_POS_4)),
                        new WaitCommand(SERVO_WAIT_TIME),

                        // 4. Lunch at POS_4
                        new InstantCommand(() -> upwardServo.setPosition(UPWARD_POS_HIGH)),
                        new WaitCommand(LUNCH_WAIT_TIME),
                        new InstantCommand(() -> upwardServo.setPosition(UPWARD_POS_LOW)),
                        new WaitCommand(SERVO_WAIT_TIME),
                        
                        // Move to POS_5
                        new InstantCommand(() -> wheelServo.setPosition(WHEEL_POS_5)),
                        new WaitCommand(SERVO_WAIT_TIME),
                        
                        // 5. Lunch at POS_5
                        new InstantCommand(() -> upwardServo.setPosition(UPWARD_POS_HIGH)),
                        new WaitCommand(LUNCH_WAIT_TIME),
                        new InstantCommand(() -> upwardServo.setPosition(UPWARD_POS_LOW)),
                        new WaitCommand(SERVO_WAIT_TIME),
                        
                        // Move to POS_6
                        new InstantCommand(() -> wheelServo.setPosition(WHEEL_POS_6)),
                        new WaitCommand(SERVO_WAIT_TIME),
                        
                        // 6. Lunch at POS_6 (Optional, 最后一下)
                        new InstantCommand(() -> upwardServo.setPosition(UPWARD_POS_HIGH)),
                        new WaitCommand(LUNCH_WAIT_TIME),
                        new InstantCommand(() -> upwardServo.setPosition(UPWARD_POS_LOW))
                )
        );
        
        // B 键停止
        new FunctionalButton(
                () -> gamepadEx.getButton(GamepadKeys.Button.B)
        ).whenPressed(
                new InstantCommand(() -> {
                    shooterUp.setPower(0);
                    shooterDown.setPower(0);
                })
        );
    }
}
