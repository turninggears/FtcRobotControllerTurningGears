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

import java.security.PublicKey;

@Config

public class Launcher {

    public static double LAUNCHER_VELOCITY = 1180;
    public static double LAUNCHER_IDLE_VELOCITY = 800;

    DcMotorEx pollenLauncher;
    DcMotorEx nectarLauncher;

    //0=stopped, 1=forward, -1 = reverse
    private int pollenLauncherDirection = 0;
    private  double lastP = Double.NaN;
    private double lastI = Double.NaN;
    private double lastD = Double.NaN;
    private double lastF = Double.NaN;

    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        PIDFCoefficients pollenLauncherPidfCoefficients = new PIDFCoefficients(PollenLauncherPIDF.P, PollenLauncherPIDF.I, PollenLauncherPIDF.D, PollenLauncherPIDF.F);
        PIDFCoefficients nectarLauncherPidfCoefficients = new PIDFCoefficients(NectarLauncherPIDF.P, NectarLauncherPIDF.I, NectarLauncherPIDF.D, NectarLauncherPIDF.F);


        pollenLauncher = hardwareMap.get(DcMotorEx.class, "Pollen Launcher");
        pollenLauncher.setDirection(DcMotorEx.Direction.FORWARD);
        pollenLauncher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        pollenLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenLauncher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pollenLauncherPidfCoefficients);

// Remember the PIDF settings already applied
        lastP = PollenLauncherPIDF.P;
        lastI = PollenLauncherPIDF.I;
        lastD = PollenLauncherPIDF.D;
        lastF = PollenLauncherPIDF.F;

        pollenLauncher.setVelocity(0);
       /* nectarLauncher = hardwareMap.get(DcMotorEx.class, "Nectar Motor");
        nectarLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        nectarLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        nectarLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, nectarLauncherPidfCoefficients);*/
    }

    //-----METHODS-----
    public void startPollenLauncher() {
        pollenLauncherDirection = 1;
        pollenLauncher.setVelocity(LAUNCHER_VELOCITY);
    }

    public void stopPollenLauncher() {
        pollenLauncherDirection = 0;
        pollenLauncher.setVelocity(0);
    }

    public void reversePollenLauncher() {
        pollenLauncherDirection = -1;
        pollenLauncher.setVelocity(-LAUNCHER_VELOCITY);
    }

    public void idlePollenLauncher() {
        pollenLauncherDirection = 1;
        pollenLauncher.setVelocity(LAUNCHER_IDLE_VELOCITY);
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

    public double getPollenLauncherVelocity() {
        return pollenLauncher.getVelocity();
    }
    public double getPollenLauncherTargetVelocity() {
            return pollenLauncherDirection * LAUNCHER_VELOCITY;
    }
    public void updatePollenLauncher() {

        // Read current PIDF values from FTC Dashboard
        double currentP = PollenLauncherPIDF.P;
        double currentI = PollenLauncherPIDF.I;
        double currentD = PollenLauncherPIDF.D;
        double currentF = PollenLauncherPIDF.F;

        // Update motor PIDF only when a value changes
        if (currentP != lastP ||
                currentI != lastI ||
                currentD != lastD ||
                currentF != lastF) {

            pollenLauncher.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(
                            currentP,
                            currentI,
                            currentD,
                            currentF
                    )
            );

            // Save the values that were applied
            lastP = currentP;
            lastI = currentI;
            lastD = currentD;
            lastF = currentF;
        }

        // Continue applying live target velocity
        pollenLauncher.setVelocity(
                pollenLauncherDirection * LAUNCHER_VELOCITY
        );
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
