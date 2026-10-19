package org.firstinspires.ftc.teamcode.System;


import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class Intake {

    public static double INTAKE_POWER = 1.0;
    DcMotorEx intakeMotor;
    Telemetry telemetry;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake Motor");
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public Action startIntake() {
        return telemetryPacket -> {
            start();
            return false;
        };
    }

    public Action stopIntake() {
        return telemetryPacket -> {
            stop();
            return false;
        };
    }

    public Action reverseIntake() {
        return telemetryPacket -> {
            reverse();
            return false;
        };
    }
}
