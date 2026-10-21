package org.firstinspires.ftc.teamcode.System;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.System.PollenLauncherPIDF;
import org.firstinspires.ftc.teamcode.System.NectarLauncherPIDF;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import org.firstinspires.ftc.robotcore.external.Telemetry;



@Config

public class Launcher {

    public static double LAUNCHER_VELOCITY = 1180;
    public static double LAUNCHER_IDLE_VELOCITY = 800;

    // Gate Servo Setting - Adjust after testing
    public static double GATE_CLOSED_POSITION = 0.0;
    public static double GATE_OPEN_POSITION = .5;

    // AUTONOMOUS GATE OPEN DURATION IN SECONDS
    public static double AUTO_GATE_OPEN_TIME = 2.0;

    Servo pollenGate;

    DcMotorEx pollenLauncher;
    DcMotorEx nectarLauncher;

    //0=stopped, 1=forward, -1 = reverse
    private int pollenLauncherDirection = 0;
    private boolean pollenLauncherIdle = false;
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

       /* save this for when pollne gate servo is set up

        pollenGate = hardwareMap.get(Servo.class, "Pollen Gate");

        closePollenGate();*/

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
        pollenLauncherIdle = false;
    }

    public void stopPollenLauncher() {
        pollenLauncherDirection = 0;
        pollenLauncherIdle = false;
        pollenLauncher.setVelocity(0);
       // closePollenGate();//save for when gate is active
    }

    public void reversePollenLauncher() {
        pollenLauncherDirection = -1;
        pollenLauncher.setVelocity(-LAUNCHER_VELOCITY);
        pollenLauncherIdle = false;
    }

    public void idlePollenLauncher() {
        pollenLauncherDirection = 1;
        pollenLauncherIdle = true;
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

    public void openPollenGate() {
        pollenGate.setPosition(GATE_OPEN_POSITION);
    }

    public void closePollenGate() {
        pollenGate.setPosition(GATE_CLOSED_POSITION);
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

        if (pollenLauncherDirection == 0) {
            return 0;
        }

        if (pollenLauncherIdle) {
            return LAUNCHER_IDLE_VELOCITY;
        }

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
        double targetVelocity;

        if (pollenLauncherDirection == 0) {
            targetVelocity = 0;
        } else if (pollenLauncherIdle) {
            targetVelocity = LAUNCHER_IDLE_VELOCITY;
        } else {
            targetVelocity = pollenLauncherDirection * LAUNCHER_VELOCITY;
        }

        pollenLauncher.setVelocity(targetVelocity);
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
    public Action LaunchFourPollenAction() {
        return new SequentialAction(
                packet -> {
                    openPollenGate();
                    return false;
                },
                new SleepAction(AUTO_GATE_OPEN_TIME),

                packet -> {
                    return false;
                }
        );
    }
}
