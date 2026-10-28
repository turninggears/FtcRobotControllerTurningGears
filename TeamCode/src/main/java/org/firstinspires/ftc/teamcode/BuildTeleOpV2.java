
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

import org.firstinspires.ftc.teamcode.System.Intake;
import org.firstinspires.ftc.teamcode.System.Launcher;
import org.firstinspires.ftc.teamcode.System.Vision;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@SuppressLint("DefaultLocale")
@TeleOp(name = "BuildTeleOpV2", group = "Test")
@Config
public class BuildTeleOpV2 extends OpMode {

    // ========================================
    // DRIVING VARIABLES
    // ========================================

    MecanumDrive drive;

    boolean fieldCentric = true;
    boolean previousTriangle = false;
    boolean previousSquare = false;
    double headingOffset = 0.0;


    // ========================================
    // VARIABLES
    // ========================================

    Launcher launcher;
    Intake intake;
    Vision vision;
// ========================================
// APRILTAG AIMING SETTINGS
// ========================================

    // Replace with actual BioBuzz cluster name
// after confirming it in telemetry.
    public static String TARGET_CLUSTER_NAME = "";

    // Intake motor state
// 0 = stopped, 1 = forward, -1 = reverse
    int intakeDirection = 0;


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

        launcher = new Launcher(hardwareMap, telemetry);
        launcher.stopPollenLauncher();

        // -------- INTAKE SETUP --------
        intake = new Intake(hardwareMap, telemetry);
        intake.stop();

        //-----VISION SETUP------
        vision = new Vision(hardwareMap);




        // -------- DASHBOARD TELEMETRY --------

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        telemetry.update();
    }


    // ========================================
    // MAIN LOOP
    // ========================================

    @Override
    public void loop() {

        //update pollen launcher Pidf and Velocity from dashboard
        launcher.updatePollenLauncher();

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

        //save this for when we add gate servo
        // Triangle: Hold to open pollen gate
// Release Triangle to close pollen gate
        if (gamepad2.triangle) {
            launcher.openPollenGate();
        } else {
            launcher.closePollenGate();
        }

        //Direction pad up or down
        if (gamepad2.dpadLeftWasPressed()){
            intake.rampsDown();
        }

        if (gamepad2.dpadRightWasPressed()){
            intake.rampsUp();
        }


// Square: Start Pollen Launcher Forward
        if (gamepad2.squareWasPressed()) {
            launcher.startPollenLauncher();
        }

// Circle: Start Pollen Launcher Reverse
        else if (gamepad2.circleWasPressed()) {
            launcher.reversePollenLauncher();
        }

// Cross: Stop Pollen Launcher
        else if (gamepad2.crossWasPressed()) {
            launcher.stopPollenLauncher();
        }

// D-Pad Up: Increase Target Velocity by 10
        else if (gamepad2.dpadUpWasPressed()) {
            Launcher.LAUNCHER_VELOCITY += 10;
        }

// D-Pad Down: Decrease Target Velocity by 10
        else if (gamepad2.dpadDownWasPressed()) {
            Launcher.LAUNCHER_VELOCITY = Math.max(
                    0,
                    Launcher.LAUNCHER_VELOCITY - 10
            );
        }
        // ====================================
// GAMEPAD 2 - INTAKE MOTOR
// ====================================

// R1: Toggle Intake REVERSE (collect elements)
        if (gamepad2.rightBumperWasPressed()) {

            if (intakeDirection == -1) {
                intakeDirection = 0;
                intake.stop();
            } else {
                intakeDirection = -1;
                intake.reverse();
            }
        }

// L1: Toggle Intake FORWARD (eject elements)
        else if (gamepad2.leftBumperWasPressed()) {

            if (intakeDirection == 1) {
                intakeDirection = 0;
                intake.stop();
            } else {
                intakeDirection = 1;
                intake.start();
            }
        }

// ====================================
// APRILTAG VISION TELEMETRY
// ====================================

        telemetry.addData(
                "Camera",
                vision.isCameraStreaming() ? "Streaming" : "Not Streaming"
        );

        telemetry.addData(
                "AprilTags Detected",
                vision.getDetections().size()
        );

        for (AprilTagDetection tag : vision.getDetections()) {

            if (tag instanceof AprilTagClusterDetection) {

                AprilTagClusterDetection cluster =
                        (AprilTagClusterDetection) tag;

                if (cluster.metadata != null) {
                    telemetry.addData(
                            "Cluster Name",
                            cluster.metadata.name
                    );

                    telemetry.addData(
                            "Cluster Short Name",
                            cluster.metadata.shortName
                    );
                }

                telemetry.addData(
                        "Cluster Found (%)",
                        cluster.percentClusterFound
                );

                if (cluster.ftcPose != null) {

                    telemetry.addData(
                            "Cluster Range",
                            "%.1f",
                            cluster.ftcPose.range
                    );

                    telemetry.addData(
                            "Cluster Bearing",
                            "%.1f",
                            cluster.ftcPose.bearing
                    );

                    telemetry.addData(
                            "Cluster Yaw",
                            "%.1f",
                            cluster.ftcPose.yaw
                    );
                }
            }
        }


        // ====================================
        // APRILTAG AIMING TELEMETRY
        // ====================================

        telemetry.addLine("");
        telemetry.addLine("APRILTAG AIMING:");

        if (!vision.isCameraStreaming()) {

            telemetry.addData("Aim Status",
                    "Camera Not Ready");

        } else if (TARGET_CLUSTER_NAME.isEmpty()) {

            telemetry.addData("Aim Status",
                    "Select Target Cluster");

        } else {

            Double aimCorrection =
                    vision.getAimCorrection(TARGET_CLUSTER_NAME);

            if (aimCorrection == null) {

                telemetry.addData("Aim Status",
                        "No Valid Target");

            } else {

                telemetry.addData(
                        "Aim Correction (deg)",
                        "%.2f",
                        aimCorrection
                );

                telemetry.addData(
                        "Aim Status",
                        vision.isAimedAtTarget(TARGET_CLUSTER_NAME)
                                ? "ALIGNED"
                                : "ADJUST HEADING"
                );
            }
        }


        // ====================================
        // TELEMETRY VELOCITY
        // ====================================
        telemetry.addData(
                "Actual Velocity",
                launcher.getPollenLauncherVelocity()
        );

        telemetry.addData(
                "Target Velocity",
                launcher.getPollenLauncherTargetVelocity()
        );
//TELEMETRY DRIVING
        telemetry.addData(
                "Drive Mode",
                fieldCentric
                        ? "Field Centric"
                        : "Robot Centric"
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
        launcher.stopPollenLauncher();

        // Stop drivetrain
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0, 0),
                        0
                )
        );
        // Stop intake motor
        intake.stop();
        intakeDirection = 0;

        //Close Webcam
        vision.close();
    }
}
