package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "ProtoTypeV2", group = "Robot")
@Config
public class ProtoTypeV2 extends OpMode {
    private DcMotorEx motor1; // Use DcMotorEx for advanced features
    private DcMotorEx motor2; // Use DcMotorEx for advanced features

    // Variable to track the current power level
    private double powerLevel = 0.0;

    // Debounced button states
    private boolean lastSquareState = false;
    private boolean lastCircleState = false;
    private boolean lastAButtonState = false;  // Bottom button
    private boolean lastTriangleState = false; // Triangle button

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotorEx.class, "launcherMotor"); // Use DcMotorEx
//        motor2 = hardwareMap.get(DcMotorEx.class, "motor2"); // Use DcMotorEx
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
//        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Optionally, set motor modes (e.g., run with encoders)
        //motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // Check for button presses with debouncing
        if (gamepad1.square && !lastSquareState) {
            powerLevel += 0.1;  // Increase power by 0.1
        }
        if (gamepad1.circle && !lastCircleState) {
            powerLevel -= 0.1;  // Decrease power by 0.1
        }
        if (gamepad1.cross && !lastAButtonState) {
            powerLevel = 0.0;  // Reset power to 0 (BOTTOM BUTTON)
        }
        if (gamepad1.triangle && !lastTriangleState) {
            powerLevel = -1.0;  // Set power to -1 (FULL REVERSE)
        }

        // Update button states for debouncing after action
        lastSquareState = gamepad1.square;
        lastCircleState = gamepad1.circle;
        lastAButtonState = gamepad1.cross;
        lastTriangleState = gamepad1.triangle;

        // Constrain the power level between -1.0 and 1.0
        powerLevel = Math.max(-1.0, Math.min(powerLevel, 1.0));

        // Set motor power based on the current power level
        motor1.setPower(powerLevel);
//        motor2.setPower(powerLevel);

        // Telemetry to display current power level and motor stats
        telemetry.addData("Current Power Level", powerLevel);
        telemetry.addData("Square Button", gamepad1.square ? "Pressed" : "Not Pressed");
        telemetry.addData("Circle Button", gamepad1.circle ? "Pressed" : "Not Pressed");
        telemetry.addData("Cross Button", gamepad1.cross ? "Pressed" : "Not Pressed");
        telemetry.addData("Triangle Button", gamepad1.triangle ? "Pressed" : "Not Pressed");

        // Motor 1 telemetry (Using DcMotorEx)
        telemetry.addData("Motor 1 Current", motor1.getCurrent(CurrentUnit.AMPS));  // Motor current in Amps
        telemetry.addData("Motor 1 Velocity", motor1.getVelocity() );  //
        telemetry.addData("Motor 1 Velocity (deg/s)", motor1.getVelocity(AngleUnit.DEGREES) );  //

        // Motor 2 telemetry (Using DcMotorEx)
//        telemetry.addData("Motor 2 Current", motor2.getCurrent(CurrentUnit.AMPS));  // Motor current in Amps

        telemetry.update();
    }
}
