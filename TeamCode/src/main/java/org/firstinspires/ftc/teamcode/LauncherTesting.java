package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.System.PIDF;

@SuppressLint("DefaultLocale")
@TeleOp(name = "LaunchingTesting", group = "Robot")
@Config

public class LauncherTesting extends OpMode{

    DcMotorEx motor;
    //Servo servo;
    //ServoController servoController;

    public static double targetVelocity = 1100;
    boolean flywheelRunning = false;



    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        //servo = hardwareMap.get(Servo.class,"servo");
        //servoController = hardwareMap.get(ServoController.class, "servocontroller");
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);

        //this was added 10/1
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );//this was end of addition from 10/1


    }

    @Override
    //This is the code that runs repeatedly once you press play. Put game play code in this section
    public void loop() {

        //Start of what i added 10/1
        motor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        PIDF.P,
                        PIDF.I,
                        PIDF.D,
                        PIDF.F
                        )
        );
       //End Of What I Added 10/1

        // servo.setPosition(0.444);
       // motor.setDirection(DcMotorSimple.Direction.FORWARD);
        //motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motor.setPower(0.349);

        //servo.setPosition(0.5);

        //motor.setDirection(DcMotor.Direction.FORWARD);

        //motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       // motor.setPower(0.5);

        // Square = Forward
        if (gamepad2.square) {
            motor.setDirection(DcMotorEx.Direction.FORWARD);
            //motor.setVelocity(targetVelocity);
            flywheelRunning = true;
        }

        // Circle = Reverse
        else if (gamepad2.circle) {
            motor.setDirection(DcMotorEx.Direction.REVERSE);
            //motor.setVelocity(targetVelocity);
            flywheelRunning = true;
        }

        //Flywheel stop command
        else if (gamepad2.crossWasPressed()) {
            motor.setDirection(DcMotorEx.Direction.FORWARD);
            flywheelRunning = false;
            motor.setVelocity(0.0);

        }

        //Velocity reduction
        else if (gamepad2.dpadDownWasPressed() /*&& motor.getPower()>0.0*/) {
            //motor.setPower(motor.getPower()-0.01);
            targetVelocity -= 10;
            motor.setVelocity(targetVelocity);
        }

        //Velocity Increase
        else if (gamepad2.dpadUpWasPressed() /*&& motor.getPower()<1.0*/) {
            //motor.setPower(motor.getPower()+0.01);
            targetVelocity += 10;
            motor.setVelocity(targetVelocity);
        }

        if (flywheelRunning){
            motor.setVelocity(targetVelocity);
        }

        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Actual Velocity", motor.getVelocity());
        telemetry.addData("Motor Power", motor.getPower());
        telemetry.addData("PIDF P", PIDF.P);
        telemetry.addData("PIDF I", PIDF.I);
        telemetry.addData("PIDF D", PIDF.D);
        telemetry.addData("PIDF F", PIDF.F);
        telemetry.update();
    }


       // if (gamepad1.a && motor.getDirection() == DcMotor.Direction.FORWARD) {
         //   motor.setDirection(DcMotor.Direction.REVERSE);
       // } else if (gamepad1.a && motor.getDirection() == DcMotor.Direction.REVERSE) {
       //     motor.setDirection(DcMotor.Direction.FORWARD);
        }
