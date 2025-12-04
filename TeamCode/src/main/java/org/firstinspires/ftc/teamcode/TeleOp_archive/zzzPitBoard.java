package org.firstinspires.ftc.teamcode.TeleOp_archive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "zzzPitBoard", group = "TeleOp")
public class zzzPitBoard extends LinearOpMode {

    private CRServo servo1;
    private CRServo servo2;
    private CRServo servo3;
    private CRServo servo4;

    @Override
    public void runOpMode() {

        // Map the servos to the names in your RC configuration
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        servo3 = hardwareMap.get(CRServo.class, "servo3");
        servo4 = hardwareMap.get(CRServo.class, "servo4");

        // Optional: set directions if some servos need to spin the opposite way
        // serv01.setDirection(CRServo.Direction.FORWARD);
        // servo2.setDirection(CRServo.Direction.REVERSE);
        // servo3.setDirection(CRServo.Direction.FORWARD);
        // servo4.setDirection(CRServo.Direction.REVERSE);

        telemetry.addLine("Initialized. Press PLAY to start servos.");
        telemetry.update();

        waitForStart();

        // Power level for continuous motion (-1.0 to 1.0)
        double servoForward = .1; // full speed forward
        double servoReverse = -.1;//half speed reverse

        while (opModeIsActive()) {
            // Keep all four servos running continuously
            servo1.setPower(servoForward);
            servo2.setPower(servoReverse);
            servo3.setPower(servoForward);
            servo4.setPower(servoReverse);

            telemetry.addData("Servo Foward Power", servoForward);
            telemetry.addData("Servo Power Reverse" , servoReverse);
            telemetry.update();
        }

        // Stop servos when OpMode ends
        servo1.setPower(0);
        servo2.setPower(0);
        servo3.setPower(0);
        servo4.setPower(0);
    }
}
