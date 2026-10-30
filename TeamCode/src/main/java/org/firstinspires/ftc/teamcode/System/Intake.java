package org.firstinspires.ftc.teamcode.System;


import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import java.security.PublicKey;

@Config
public class Intake {

    public static double INTAKE_POWER = 1.0;

    public static double RIGHT_RAMP_UP = 1;
    public static double LEFT_RAMP_UP = 0;

    public static double RIGHT_RAMP_DOWN = 0;

    public static double LEFT_RAMP_DOWN = .3;

    Servo rightRamp;
    Servo leftRamp;
    DcMotorEx intakeMotor;
    Telemetry telemetry;


    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake Motor");
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRamp = hardwareMap.get(Servo.class, "Right Ramp");
        leftRamp = hardwareMap.get(Servo.class, "Left Ramp");

        //defaults both ramps up
        rampsUp();
    }



    public void start() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }

    public void reverse() {
        intakeMotor.setPower(-INTAKE_POWER);
    }




    public Action startIntakeAction() {
        return telemetryPacket -> {
            start();
            return false;
        };
    }


    public Action stopIntakeAction() {
        return telemetryPacket -> {
            stop();
            return false;
        };
    }

    public Action reverseIntakeAction() {
        return telemetryPacket -> {
            reverse();
            return false;
        };
    }


    public Action rampsUpAction() {
        return telemetryPacket -> {
            rampsUp();
            return false;
        };
    }

    public Action rampsDownAction() {
        return telemetryPacket -> {
            rampsDown();
            return false;
        };
    }


    public void rampsUp() {
        rightRamp.setPosition(RIGHT_RAMP_UP);
        leftRamp.setPosition(LEFT_RAMP_UP);
    }

    public void rampsDown() {
        rightRamp.setPosition(RIGHT_RAMP_DOWN);
        leftRamp.setPosition(LEFT_RAMP_DOWN);
    }


}
