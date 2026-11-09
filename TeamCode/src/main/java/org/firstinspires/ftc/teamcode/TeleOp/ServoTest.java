
package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressLint("DefaultLocale")
@TeleOp(name = "Servo Test", group = "Test")
@Config
public class ServoTest extends OpMode {

    // ========================================
    // INITIALIZATION
    // ========================================

    Servo rightRamp;
    Servo leftRamp;

    public static double MIN_POSITION = 0.0;
    public static double MAX_POSITION = 1.0;
    public static double POSITION_INCREMENT = 0.05;

    public static double left_target_position = 0.0;
    public static double right_target_position = 0.0;

    double left_current_position;
    double right_current_position;

    @Override
    public void init() {

        rightRamp = hardwareMap.get(Servo.class, "Right Ramp");
        leftRamp = hardwareMap.get(Servo.class, "Left Ramp");

        rightRamp.setDirection(Servo.Direction.REVERSE);
        leftRamp.setDirection(Servo.Direction.FORWARD);

        rightRamp.setPosition(MIN_POSITION);
        leftRamp.setPosition(MIN_POSITION);


        left_current_position = leftRamp.getPosition();
        right_current_position= rightRamp.getPosition();

        telemetry.addData("Left Ramp position: ", left_current_position);
        telemetry.addData("Right Ramp position: ", right_current_position);
        telemetry.update();
    }


    // ========================================
    // MAIN LOOP
    // ========================================

    @Override
    public void loop() {

        telemetry.addData("Left Ramp position: ", left_current_position);
        telemetry.addData("Right Ramp position: ", right_current_position);

        // ====================================
        // GAMEPAD 2 - Servo Tester
        // ====================================

        if (gamepad2.triangle && rightRamp.getPosition() < MAX_POSITION) {
            right_current_position = rightRamp.getPosition();
            rightRamp.setPosition(right_current_position + POSITION_INCREMENT);
        }

        if (gamepad2.cross && rightRamp.getPosition() > MIN_POSITION) {
            right_current_position = rightRamp.getPosition();
            rightRamp.setPosition(right_current_position - POSITION_INCREMENT);
        }

        if (gamepad2.circle) {
            rightRamp.setPosition(right_target_position);
        }

        if (gamepad2.dpad_up && leftRamp.getPosition() < MAX_POSITION) {
            left_current_position = leftRamp.getPosition();
            leftRamp.setPosition(left_current_position + POSITION_INCREMENT);
        }

        if (gamepad2.dpad_down && leftRamp.getPosition() > MIN_POSITION) {
            left_current_position = leftRamp.getPosition();
            leftRamp.setPosition(left_current_position - POSITION_INCREMENT);
        }

        if (gamepad2.dpad_right) {
            leftRamp.setPosition(left_target_position);
        }

        // ====================================
        // CONTROLLER INSTRUCTIONS
        // ====================================

        telemetry.addLine("");
        telemetry.addLine("GAMEPAD 2:");
        telemetry.addLine("Triangle: step Right Ramp up");
        telemetry.addLine("Cross: step Right Ramp down");
        telemetry.addLine("Circle: set Right Ramp position to right_target_position");
        telemetry.addLine("D-pad up: step Left Ramp up");
        telemetry.addLine("D-pad down: step Left Ramp down");
        telemetry.addLine("D-pad right: set Left Ramp to left_target_position");

        telemetry.update();
    }

}
