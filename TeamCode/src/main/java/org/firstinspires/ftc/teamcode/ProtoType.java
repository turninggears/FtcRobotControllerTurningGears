package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Proto Type", group = "Robot")
    @Config

    public class ProtoType extends OpMode {

        // This declares the motors needed
        DcMotor motor1;
        DcMotor motor2;


        @Override
        public void init() {
            motor1 = hardwareMap.get(DcMotor.class, "motor1");
            motor2 = hardwareMap.get(DcMotor.class, "motor2");


         /* motor1.setDirection(DcMotor.Direction.REVERSE);
         motor2.setDirection(DcMotor.Direction.FORWARD);
*/

        }

        @Override
        public void loop() {
            //activate these to add messages to control hub
            //   telemetry.addLine("add text");
            //   telemetry.addLine("add text");
            //   telemetry.addLine("add text");
            //   telemetry.addLine("add text");


            if (gamepad1.circle) {
                motor1.setDirection(DcMotorSimple.Direction.FORWARD);
                motor2.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            if (gamepad1.square) {
                motor1.setDirection(DcMotorSimple.Direction.REVERSE);
                motor2.setDirection(DcMotorSimple.Direction.REVERSE);
            }

        }
    }
