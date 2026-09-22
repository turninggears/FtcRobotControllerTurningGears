package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.System.PIDF;

@SuppressLint("DefaultLocale")
@TeleOp(name = "TeleOpCompetition", group = "Robot")
@Config

public class TeleOpCompetition extends OpMode {

    DcMotorEx motor;
    //Servo servo;
    //ServoController servoController;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        //servo = hardwareMap.get(Servo.class,"servo");
        //servoController = hardwareMap.get(ServoController.class, "servocontroller");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0
        );

    }

    @Override
    //This is the code that runs repeatedly once you press play. Put game play code in this section
    public void loop() {
       // servo.setPosition(0.444);
       // motor.setDirection(DcMotorSimple.Direction.FORWARD);
        //motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motor.setPower(0.349);

        //servo.setPosition(0.5);

        //motor.setDirection(DcMotor.Direction.FORWARD);

        //motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       // motor.setPower(0.5);

        // A = Forward
        if (gamepad1.square) {
            motor.setDirection(DcMotorEx.Direction.FORWARD);
            motor.setPower(0.5);
        }

        // B = Reverse
        else if (gamepad1.circle) {
            motor.setDirection(DcMotorEx.Direction.REVERSE);
            motor.setPower(0.5);
        }

        // Square = Stop
        else if (gamepad1.x) {
            motor.setPower(0);
        }
    }

       // if (gamepad1.a && motor.getDirection() == DcMotor.Direction.FORWARD) {
         //   motor.setDirection(DcMotor.Direction.REVERSE);
       // } else if (gamepad1.a && motor.getDirection() == DcMotor.Direction.REVERSE) {
       //     motor.setDirection(DcMotor.Direction.FORWARD);
        }
