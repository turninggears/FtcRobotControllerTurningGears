package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Proto Type", group = "Robot")
    @Config

    public class ProtoType extends OpMode {

        // This declares the motors needed
        DcMotor launcherMotor;
        DcMotor motor2;


        @Override
        public void init() {
            launcherMotor = hardwareMap.get(DcMotor.class, "launcherMotor");
//            motor2 = hardwareMap.get(DcMotor.class, "motor2");


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
                launcherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//                motor2.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            if (gamepad1.square) {
                launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//                motor2.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            if (gamepad2.left_bumper) {
                launcherMotor.setPower(1);
                ((DcMotorEx) launcherMotor).setVelocity(1440);
            } else {
                launcherMotor.setPower(0);
            }

        }
    }
