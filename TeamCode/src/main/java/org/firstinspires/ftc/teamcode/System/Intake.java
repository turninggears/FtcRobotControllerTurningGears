
package org.firstinspires.ftc.teamcode.System;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {

    public static double INTAKE_POWER = 1.0;
    public static double FRONT_INTAKE_POWER = 1.0;

    public static double RAMP_UP = 0.73;
    public static double RAMP_DOWN = 0;

    public static double RAMP_SHAKE = .15;

    Servo rightRamp;
    Servo leftRamp;

    DcMotorEx intakeMotor;
    DcMotorEx frontIntakeMotor;

    Telemetry telemetry;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        // Main intake motor
        intakeMotor = hardwareMap.get(
                DcMotorEx.class, "Intake Motor");

        intakeMotor.setDirection(
                DcMotorEx.Direction.FORWARD);

        intakeMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        // Front intake motor
        frontIntakeMotor = hardwareMap.get(
                DcMotorEx.class, "Front Intake");

        frontIntakeMotor.setDirection(
                DcMotorEx.Direction.REVERSE);

        frontIntakeMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        frontIntakeMotor.setPower(0);

        // Ramp servos
        rightRamp = hardwareMap.get(
                Servo.class, "Right Ramp");

        leftRamp = hardwareMap.get(
                Servo.class, "Left Ramp");

        rightRamp.setDirection(Servo.Direction.REVERSE);
        leftRamp.setDirection(Servo.Direction.FORWARD);

        // Default both ramps up
        rampsUp();
    }

    // =====================================
    // MAIN INTAKE MOTOR
    // =====================================

    public void start() {
        intakeMotor.setPower(-INTAKE_POWER);
        frontIntakeMotor.setPower(FRONT_INTAKE_POWER);
    }

    public void stop() {
        intakeMotor.setPower(0);
        frontIntakeMotor.setPower(0);
    }

    public void reverse() {
        intakeMotor.setPower(INTAKE_POWER);
        frontIntakeMotor.setPower(-FRONT_INTAKE_POWER);
    }

    // =====================================
    // FRONT INTAKE MOTOR
    // =====================================

    public void startFrontIntake() {
        frontIntakeMotor.setPower(FRONT_INTAKE_POWER);
    }

    public void reverseFrontIntake() {
        frontIntakeMotor.setPower(-FRONT_INTAKE_POWER);
    }

    public void stopFrontIntake() {
        frontIntakeMotor.setPower(0);
    }

    // =====================================
    // MAIN INTAKE ACTIONS
    // =====================================

    public Action startIntakeAction() {
        return packet -> {
            start();
            return false;
        };
    }

    public Action stopIntakeAction() {
        return packet -> {
            stop();
            return false;
        };
    }

    public Action reverseIntakeAction() {
        return packet -> {
            reverse();
            return false;
        };
    }

    // =====================================
    // FRONT INTAKE ACTIONS
    // =====================================

    public Action startFrontIntakeAction() {
        return packet -> {
            startFrontIntake();
            return false;
        };
    }

    public Action reverseFrontIntakeAction() {
        return packet -> {
            reverseFrontIntake();
            return false;
        };
    }

    public Action stopFrontIntakeAction() {
        return packet -> {
            stopFrontIntake();
            return false;
        };
    }

    // =====================================
    // RAMP ACTIONS
    // =====================================

    public Action rampsUpAction() {
        return packet -> {
            rampsUp();
            return false;
        };
    }

    public Action rampsDownAction() {
        return packet -> {
            rampsDown();
            return false;
        };
    }

    // =====================================
    // RAMP CONTROL
    // =====================================

    public void rampsUp() {
        rightRamp.setPosition(RAMP_UP);
        leftRamp.setPosition(RAMP_UP);
    }

    public void rampsDown() {
        rightRamp.setPosition(RAMP_DOWN);
        leftRamp.setPosition(RAMP_DOWN);
    }

    public Action rampsShakeAction() {
        return new SequentialAction(
                packet -> {
                    rightRamp.setPosition(RAMP_SHAKE);
                    leftRamp.setPosition(RAMP_SHAKE);
                    return false;
                },
                new SleepAction(0.15),

                packet -> {
                    rampsDown();
                    return false;
                },
                new SleepAction(0.15),

                packet -> {
                    rightRamp.setPosition(RAMP_SHAKE);
                    leftRamp.setPosition(RAMP_SHAKE);
                    return false;
                },
                new SleepAction(0.15),

                packet -> {
                    rampsDown();
                    return false;
                },
                new SleepAction(0.15),

                packet -> {
                    rightRamp.setPosition(RAMP_SHAKE);
                    leftRamp.setPosition(RAMP_SHAKE);
                    return false;
                },
                new SleepAction(0.15),

                packet -> {
                    rampsDown();
                    return false;
                }
        );
    }
}
