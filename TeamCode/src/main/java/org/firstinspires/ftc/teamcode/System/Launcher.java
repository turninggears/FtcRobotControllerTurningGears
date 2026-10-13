package org.firstinspires.ftc.teamcode.System;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.System.PIDF;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config

public class Launcher {

    public static double LAUNCHER_VELOCITY = 1200;

    public PIDFCoefficients pidfCoefficients = new PIDFCoefficients(
            PIDF.P,
            PIDF.I,
            PIDF.D,
            PIDF.F
    );
    DcMotorEx pollenLauncher;
    DcMotorEx nectarLauncher;

    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        pollenLauncher = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        pollenLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        pollenLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pollenLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        nectarLauncher = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        nectarLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        nectarLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        nectarLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }
}
