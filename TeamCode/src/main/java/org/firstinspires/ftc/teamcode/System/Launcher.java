package org.firstinspires.ftc.teamcode.System;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Launcher {
    DcMotorEx pollenLauncher;
    DcMotorEx nectarLauncher;

    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        pollenLauncher = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        pollenLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        pollenLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        nectarLauncher = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        nectarLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        nectarLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
