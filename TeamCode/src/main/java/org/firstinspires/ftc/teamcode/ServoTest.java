
package org.firstinspires.ftc.teamcode;

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

    double START_POSITION = 0.0;
    double STOP_POSITION = 1.0;

    double POSITION_INCREMENT = 0.05;
    @Override
    public void init() {

        rightRamp = hardwareMap.get(Servo.class, "Right Ramp");
        leftRamp = hardwareMap.get(Servo.class, "Left Ramp");

        rightRamp.setPosition(START_POSITION);
        rightRamp.setDirection(Servo.Direction.REVERSE);
        leftRamp.setPosition(START_POSITION);
        leftRamp.setDirection(Servo.Direction.FORWARD);
    }


    // ========================================
    // MAIN LOOP
    // ========================================

    @Override
    public void loop() {

        // ====================================
        // GAMEPAD 2 - Servo Tester
        // ====================================

        if (gamepad2.triangle && rightRamp.getPosition() < STOP_POSITION) {
            rightRamp.setPosition(rightRamp.getPosition() + POSITION_INCREMENT);
        }

        if (gamepad2.cross && rightRamp.getPosition() > START_POSITION) {
            rightRamp.setPosition(rightRamp.getPosition() - POSITION_INCREMENT);
        }

        if (gamepad2.dpad_up && leftRamp.getPosition() < STOP_POSITION) {
            leftRamp.setPosition(leftRamp.getPosition() + POSITION_INCREMENT);
        }

        if (gamepad2.dpad_down && leftRamp.getPosition() > START_POSITION) {
            leftRamp.setPosition(leftRamp.getPosition() - POSITION_INCREMENT);
        }

        // ====================================
        // CONTROLLER INSTRUCTIONS
        // ====================================

        telemetry.addLine("");
        telemetry.addLine("GAMEPAD 2:");
        telemetry.addLine("Triangle: step Right Ramp up");
        telemetry.addLine("Cross: step Right Ramp down");
        telemetry.addLine("D-pad up: step Left Ramp up");
        telemetry.addLine("D-pad down: step Left Ramp down");

        telemetry.update();
    }

}
