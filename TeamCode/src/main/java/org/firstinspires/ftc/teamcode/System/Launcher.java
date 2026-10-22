package org.firstinspires.ftc.teamcode.System;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.System.PollenLauncherPIDF;
import org.firstinspires.ftc.teamcode.System.NectarLauncherPIDF;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config

public class Launcher {

    public static double LAUNCHER_VELOCITY = 1200;

    DcMotorEx pollenLauncher;
    DcMotorEx nectarLauncher;

    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        PIDFCoefficients pollenLauncherPidfCoefficients = new PIDFCoefficients(PollenLauncherPIDF.P, PollenLauncherPIDF.I, PollenLauncherPIDF.D, PollenLauncherPIDF.F);
        PIDFCoefficients nectarLauncherPidfCoefficients = new PIDFCoefficients(NectarLauncherPIDF.P, NectarLauncherPIDF.I, NectarLauncherPIDF.D, NectarLauncherPIDF.F);


        pollenLauncher = hardwareMap.get(DcMotorEx.class, "pollenLauncher");
        pollenLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        pollenLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pollenLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pollenLauncherPidfCoefficients);

        nectarLauncher = hardwareMap.get(DcMotorEx.class, "nectarLauncher");
        nectarLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        nectarLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        nectarLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, nectarLauncherPidfCoefficients);
    }

    public void startPollenLauncher() {
        pollenLauncher.setVelocity(LAUNCHER_VELOCITY);
    }

    public void stopPollenLauncher() {
        pollenLauncher.setVelocity(0);
    }

    public void reversePollenLauncher() {
        pollenLauncher.setVelocity(-LAUNCHER_VELOCITY);
    }

    public void increasePollenLauncherVelocity(double increment) {
        pollenLauncher.setVelocity(pollenLauncher.getVelocity()+increment);
    }

    public Action startPollenLauncherAction() {
        return telemetryPacket -> {
            startPollenLauncher();
            return false;
        };
    }

    public Action stopPollenLauncherAction() {
        return telemetryPacket -> {
            stopPollenLauncher();
            return false;
        };
    }

    public Action reversePollenLauncherAction() {
        return telemetryPacket -> {
            reversePollenLauncher();
            return false;
        };
    }

    public void startNectarLauncher() {
        nectarLauncher.setVelocity(LAUNCHER_VELOCITY);
    }

    public void stopNectarLauncher() {
        nectarLauncher.setVelocity(0);
    }

    public void reverseNectarLauncher() {
        nectarLauncher.setVelocity(-LAUNCHER_VELOCITY);
    }

    public Action startNectarLauncherAction() {
        return telemetryPacket -> {
            startNectarLauncher();
            return false;
        };
    }

    public Action stopNectarLauncherAction() {
        return telemetryPacket -> {
            stopNectarLauncher();
            return false;
        };
    }

    public Action reverseNectarLauncherAction() {
        return telemetryPacket -> {
            reverseNectarLauncher();
            return false;
        };
    }
}
