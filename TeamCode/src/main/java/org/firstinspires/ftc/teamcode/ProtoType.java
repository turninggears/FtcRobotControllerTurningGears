package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

    @TeleOp(name = "ProtoType", group = "Robot")
    @Config

    public class ProtoType extends OpMode {

        // This declares the motors needed
        DcMotor motor1;
        DcMotor motor2;


        // This declares the IMU needed to get the current direction the robot is facing
        IMU imu;


        @Override
        public void init() {
            motor1 = hardwareMap.get(DcMotor.class, "motor1");
            motor2 = hardwareMap.get(DcMotor.class, "motor2");

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


         /* motor1.setDirection(DcMotor.Direction.REVERSE);
         motor2.setDirection(DcMotor.Direction.FORWARD);
*/

            imu = hardwareMap.get(IMU.class, "imu");
            // This needs to be changed to match the orientation on your robot
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                    RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

            RevHubOrientationOnRobot orientationOnRobot = new
                    RevHubOrientationOnRobot(logoDirection, usbDirection);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
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
