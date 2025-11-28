package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.wheel.WheelConstants;

@Config
@TeleOp(name = "WheelPosTuner", group = "Test")
public class WheelPosTuner extends LinearOpMode {

    // 可以在 Dashboard 上实时修改这两个值
    // Start from the highest value (0.980)
    public static double START_POS = 0.980; 
    public static double OFFSET = 0.074;    // 每一格的间距

    public static double UP_HIGH = 0.6;
    public static double UP_LOW = 0.2;

    private Servo wheelServo;
    private Servo upwardServo;

    private int currentSlot = 0;
    private boolean lastUp = false;
    private boolean lastDown = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        wheelServo = hardwareMap.get(Servo.class, WheelConstants.wheelServoName);
        upwardServo = hardwareMap.get(Servo.class, WheelConstants.upwardServoName);

        wheelServo.setPosition(START_POS);
        upwardServo.setPosition(UP_LOW);

        telemetry.addLine("Ready. Use Dashboard to tune START_POS and OFFSET.");
        telemetry.addLine("D-Pad Up/Down to change slots (0-13).");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. 切换槽位逻辑
            boolean currentUp = gamepad1.dpad_up;
            boolean currentDown = gamepad1.dpad_down;

            if (currentDown && !lastDown) {
                currentSlot++;
                if (currentSlot > 13) currentSlot = 0; // 0-13 (14 slots)
            }
            if (currentUp && !lastUp) {
                currentSlot--;
                if (currentSlot < 0) currentSlot = 13; // 循环
            }

            lastUp = currentUp;
            lastDown = currentDown;

            // 2. 计算当前目标位置
            // 公式: 初始位置 - (第几格 * 间距)
            double targetPos = START_POS - (currentSlot * OFFSET);

            // 3. 设置舵机
            wheelServo.setPosition(targetPos);

            // 4. Test Launch (Press A to raise, release to lower)
            if (gamepad1.a) {
                upwardServo.setPosition(UP_HIGH);
            } else {
                upwardServo.setPosition(UP_LOW);
            }

            // 5. 显示所有计算结果 (方便你复制)
            telemetry.addData("Current Slot", currentSlot);
            telemetry.addData("Current Pos", String.format("%.3f", targetPos));
            telemetry.addLine("-----------------------");
            telemetry.addLine("CALCULATED POSITIONS:");
            
            double[] allPositions = new double[14]; // 14个位置
            for (int i = 0; i < 14; i++) {
                allPositions[i] = START_POS - (i * OFFSET);
            }
            telemetry.addData("Slot " + currentSlot, String.format("%.3f", allPositions[currentSlot]));
            
            // 格式化数组字符串，方便直接复制到代码里
            StringBuilder arrayStr = new StringBuilder("{");
            for(int i=0; i<14; i++) {
                arrayStr.append(String.format("%.3f", allPositions[i]));
                if(i < 13) arrayStr.append(", ");
            }
            arrayStr.append("}");
            telemetry.addData("Array Code", arrayStr.toString());

            telemetry.update();
        }
    }
}
