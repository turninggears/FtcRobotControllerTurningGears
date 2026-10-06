package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@SuppressLint("DefaultLocale")
@TeleOp(name = "IntakeControls", group = "Robot")
@Config

public class IntakeControls extends OpMode{

    DcMotorEx intakeMotor;
    Servo rightFlowerWheel;
    Servo leftFlowerWheel;



    @Override
    public void init(){

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        rightFlowerWheel = hardwareMap.get(Servo.class, "rightFlowerWheel");
        leftFlowerWheel = hardwareMap.get(Servo.class, "leftFlowerWheel");


    }

    @Override
    public void loop(){

    }
}
