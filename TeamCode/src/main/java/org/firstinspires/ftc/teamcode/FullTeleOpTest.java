
package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.System.PIDF;

@SuppressLint("DefaultLocale")
@TeleOp(name = "FullTeleOpTest", group = "Test")
@Config
public class FullTeleOpTest extends OpMode {

    // ========================================
    // DRIVING VARIABLES
    // ========================================

    MecanumDrive drive;

    boolean fieldCentric = true;
    boolean previousTriangle = false;
    boolean previousSquare = false;
    double headingOffset = 0.0;


    // ========================================
    // POLLEN LAUNCHER VARIABLES
    // ========================================

    DcMotorEx pollenLauncher;
    DcMotorEx intakeMotor;

    // Intake motor state
// 0 = stopped, 1 = forward, -1 = reverse
    int intakeDirection = 0;

    // Adjustable through FTC Dashboard
    public static double targetVelocity = 1100;

    boolean flywheelRunning = false;


    // ========================================
    // INITIALIZATION
    // ========================================

    @Override
    public void init() {

        // -------- DRIVETRAIN SETUP --------

        Pose2d startPose = new Pose2d(
                0,
                0,
                0
        );

        drive = new MecanumDrive(
                hardwareMap,
                startPose
        );


        // -------- POLLEN LAUNCHER SETUP --------

        pollenLauncher = hardwareMap.get(
                DcMotorEx.class,
                "Pollen Launcher"
        );

        pollenLauncher.setDirection(
                DcMotorEx.Direction.FORWARD
        );

        pollenLauncher.setZeroPowerBehavior(
                DcMotorEx.ZeroPowerBehavior.BRAKE
        );

        pollenLauncher.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        pollenLauncher.setPower(0);

        // intake motor setup

        intakeMotor = hardwareMap.get(
                DcMotorEx.class,
                        "Intake Motor"
        );

        intakeMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setPower(0);




        // -------- DASHBOARD TELEMETRY --------

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        telemetry.addLine("Full TeleOp Initialized");
        telemetry.update();
    }


    // ========================================
    // MAIN LOOP
    // ========================================

    @Override
    public void loop() {

        // ====================================
        // GAMEPAD 1 - DRIVING
        // ====================================

        drive.updatePoseEstimate();

        double currentHeading =
                drive.localizer.getPose()
                        .heading.toDouble();


        // Triangle: Toggle Field/Robot Centric
        if (gamepad1.triangle && !previousTriangle) {
            fieldCentric = !fieldCentric;
        }

        previousTriangle = gamepad1.triangle;


        // Square: Reset Field Heading
        if (gamepad1.square && !previousSquare) {
            headingOffset = currentHeading;
        }

        previousSquare = gamepad1.square;


        // Joystick Input
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;


        // R1: 50% Speed
        double speedMultiplier =
                gamepad1.right_bumper ? 0.5 : 1.0;


        // Drive Vector
        Vector2d driveVector =
                new Vector2d(forward, strafe);


        // Field Centric Conversion
        if (fieldCentric) {

            double driveHeading =
                    currentHeading - headingOffset;

            driveVector = Rotation2d
                    .fromDouble(-driveHeading)
                    .times(driveVector);
        }


        // Apply Drive Powers
        drive.setDrivePowers(
                new PoseVelocity2d(
                        driveVector.times(speedMultiplier),
                        turn * speedMultiplier
                )
        );


        // ====================================
        // GAMEPAD 2 - POLLEN LAUNCHER
        // ====================================

        // Apply PIDF Settings
        pollenLauncher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        PIDF.P,
                        PIDF.I,
                        PIDF.D,
                        PIDF.F
                )
        );


        // Square: Start Flywheel Forward
        if (gamepad2.squareWasPressed()) {

            pollenLauncher.setDirection(
                    DcMotorEx.Direction.FORWARD
            );

            flywheelRunning = true;
        }


        // Circle: Start Flywheel Reverse
        else if (gamepad2.circleWasPressed()) {

            pollenLauncher.setDirection(
                    DcMotorEx.Direction.REVERSE
            );

            flywheelRunning = true;
        }


        // Cross: Stop Flywheel
        else if (gamepad2.crossWasPressed()) {

            flywheelRunning = false;
            pollenLauncher.setVelocity(0);
        }


        // D-Pad Down: Decrease Velocity
        else if (gamepad2.dpadDownWasPressed()) {

            targetVelocity = Math.max(
                    0,
                    targetVelocity - 10
            );
        }


        // D-Pad Up: Increase Velocity
        else if (gamepad2.dpadUpWasPressed()) {

            targetVelocity += 10;
        }


        // Run Flywheel at Target Velocity
        if (flywheelRunning) {

            pollenLauncher.setVelocity(targetVelocity);

        } else {

            pollenLauncher.setVelocity(0);
        }


        // ====================================
        // GAMEPAD 2 - INTAKE MOTOR
        // ====================================

        // R1: Toggle Intake eject
        if (gamepad2.rightBumperWasPressed()) {

            if (intakeDirection == 1) {
                intakeDirection = 0;  // Stop
            } else {
                intakeDirection = 1;  // Forward
            }
        }

        // L1: Toggle Intake
        else if (gamepad2.leftBumperWasPressed()) {

            if (intakeDirection == -1) {
                intakeDirection = 0;  // Stop
            } else {
                intakeDirection = -1; // Reverse
            }
        }

        // Apply intake motor power
        intakeMotor.setPower(intakeDirection);

        // ====================================
        // TELEMETRY - DRIVING
        // ====================================



        telemetry.addData(
                "Drive Mode",
                fieldCentric
                        ? "Field Centric"
                        : "Robot Centric"
        );

                // ====================================
        // TELEMETRY - POLLEN LAUNCHER
        // ====================================


        telemetry.addData(
                "Target Velocity",
                targetVelocity
        );

        telemetry.addData(
                "Actual Velocity",
                pollenLauncher.getVelocity()
        );



        // ====================================
        // CONTROLLER INSTRUCTIONS
        // ====================================

        telemetry.addLine("");
        telemetry.addLine("GAMEPAD 1:");
        telemetry.addLine("Triangle = Field/Robot Centric");
        telemetry.addLine("Square = Reset Field Heading");
        telemetry.addLine("Hold R1 = 50% Speed");


        telemetry.update();
    }


    // ========================================
    // STOP - SAFETY
    // ========================================

    @Override
    public void stop() {

        // Stop pollen launcher
        pollenLauncher.setVelocity(0);
        pollenLauncher.setPower(0);

        // Stop drivetrain
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0, 0),
                        0
                )
        );
        // Stop intake motor
        intakeMotor.setPower(0);
        intakeDirection = 0;
    }
}
