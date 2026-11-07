
package org.firstinspires.ftc.teamcode.Autonomous.LegacyAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDriveBiobuzz;
import org.firstinspires.ftc.teamcode.System.Intake;
import org.firstinspires.ftc.teamcode.System.Launcher;
import org.firstinspires.ftc.teamcode.System.MatchSettings;
import org.firstinspires.ftc.teamcode.System.Vision;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "BioBuzz Red Left A", group = "Autonomous")
public class BioBuzzRedLeft_A extends LinearOpMode {

    // ========================================
    // AUTONOMOUS SETTINGS
    // ========================================

    public static double AUTO_PARK_TRIGGER_SECONDS = 23.0;

    // Require three consecutive AprilTag detections.
    public static int REQUIRED_TAG_DETECTIONS = 3;

    // ========================================
    // STOP MECHANISMS
    // ========================================

    private void stopMechanisms(
            Launcher launcher,
            Intake intake
    ) {

        launcher.stopPollenLauncher();
        intake.stop();
    }

    // ========================================
    // 25-SECOND EMERGENCY PARKING
    // ========================================

    private Action parkAtTimeout(
            MecanumDriveBiobuzz drive,
            Launcher launcher,
            Intake intake,
            Action normalSequence,
            Vector2d parkingPosition,
            double parkingHeadingDegrees
    ) {

        return new Action() {

            private Action emergencyParking = null;
            private boolean parkingStarted = false;

            @Override
            public boolean run(TelemetryPacket packet) {

                double elapsed = getRuntime();

                packet.put("Auto Elapsed", elapsed);
                packet.put("Emergency Parking", parkingStarted);

                // Interrupt normal autonomous at 25 seconds.
                if (!parkingStarted
                        && elapsed >= AUTO_PARK_TRIGGER_SECONDS) {

                    parkingStarted = true;

                    // Stop launcher and intake before parking.
                    stopMechanisms(launcher, intake);

                    // Update the robot's current position.
                    drive.updatePoseEstimate();

                    Pose2d currentPose =
                            drive.localizer.getPose();

                    // Create a trajectory from the robot's
                    // current position directly to Park B.
                    emergencyParking =
                            drive.actionBuilder(currentPose)
                                    .strafeToLinearHeading(
                                            parkingPosition,
                                            Math.toRadians(
                                                    parkingHeadingDegrees
                                            )
                                    )
                                    .build();

                    packet.put(
                            "Emergency Parking Status",
                            "TRIGGERED"
                    );
                }

                // Emergency parking replaces normal autonomous.
                if (parkingStarted) {

                    packet.put(
                            "Emergency Parking Status",
                            "PARKING"
                    );

                    return emergencyParking.run(packet);
                }

                // Continue normal autonomous before 25 seconds.
                return normalSequence.run(packet);
            }
        };
    }

    // ========================================
    // WAIT FOR RED ALLIANCE APRILTAG
    // ========================================

    private Action waitForAllianceAprilTag(Vision vision) {

        return new Action() {

            private int consecutiveDetections = 0;

            @Override
            public boolean run(TelemetryPacket packet) {

                // MatchSettings is set to RED.
                boolean tagVisible =
                        vision.isAllianceClusterVisible();

                if (tagVisible) {
                    consecutiveDetections++;
                } else {
                    consecutiveDetections = 0;
                }

                packet.put("AprilTag Visible", tagVisible);

                packet.put(
                        "Consecutive Detections",
                        consecutiveDetections
                );

                // Continue only after three consecutive
                // successful AprilTag checks.
                if (consecutiveDetections >= REQUIRED_TAG_DETECTIONS) {

                    packet.put(
                            "AprilTag Status",
                            "DETECTED"
                    );

                    return false;
                }

                packet.put(
                        "AprilTag Status",
                        "WAITING"
                );

                // No individual AprilTag timeout.
                // Overall 25-second failsafe handles parking.
                return true;
            }
        };
    }

    // ========================================
    // CONTINUOUS HEADING MONITOR
    // ========================================

    public Action monitorHeadingContinuous(
            MecanumDriveBiobuzz drive,
            boolean[] driveFinished
    ) {

        return packet -> {

            if (driveFinished[0]) {
                return false;
            }

            Pose2d actualPose =
                    drive.localizer.getPose();

            double actualHeading =
                    Math.toDegrees(
                            actualPose.heading.toDouble()
                    );

            // Normalize heading to 0-360 degrees.
            actualHeading =
                    (actualHeading % 360 + 360) % 360;

            telemetry.addData(
                    "Actual Heading (deg)",
                    "%.1f",
                    actualHeading
            );

            telemetry.update();

            packet.put(
                    "Actual Heading (deg)",
                    actualHeading
            );

            return true;
        };
    }

    // ========================================
    // MAIN AUTONOMOUS
    // ========================================

    @Override
    public void runOpMode() {

        // ========================================
        // INITIALIZE HARDWARE
        // ========================================

        Pose2d startPose = new Pose2d(
                -63.5,
                -10.5,
                Math.toRadians(180)
        );

        MecanumDriveBiobuzz drive =
                new MecanumDriveBiobuzz(
                        hardwareMap,
                        startPose
                );

        Launcher launcher =
                new Launcher(hardwareMap, telemetry);

        Intake intake =
                new Intake(hardwareMap, telemetry);

        // Default to RED alliance.
        MatchSettings.setAlliance(
                MatchSettings.Alliance.RED
        );

        // Initialize Vision after selecting alliance.
        Vision vision = new Vision(hardwareMap);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        telemetry.addData("Alliance", "RED");
        telemetry.addData("Autonomous", "Red Left A");
        telemetry.addLine("Waiting for START");
        telemetry.update();

        // ========================================
        // FIRST LAUNCH POSITION
        // ========================================

        TrajectoryActionBuilder launchPositionB =
                drive.actionBuilder(startPose)
                        .setTangent(Math.toRadians(0))
                        .strafeTo(
                                new Vector2d(-60, -10.5)
                        )
                        .waitSeconds(.75)
                        .endTrajectory();

        // ========================================
        // FLOWER LINEUP
        // ========================================

        TrajectoryActionBuilder flowerBLineUp =
                launchPositionB.fresh()
                        .strafeTo(
                                new Vector2d(-50, -28)
                        )
                        .endTrajectory();

        // ========================================
        // FLOWER PICKUP
        // ========================================

        TrajectoryActionBuilder flowerBPickup =
                flowerBLineUp.fresh()
                        .waitSeconds(.75)
                        .setTangent(Math.toRadians(180))
                        .strafeTo(
                                new Vector2d(-61.5, -28)
                        )
                        //.strafeTo(new Vector2d(-58,-28))
                        //.strafeTo(new Vector2d(-59,-28))
                        .waitSeconds(1)
                        .endTrajectory();

        // ========================================
        // SECOND LAUNCH POSITION
        // ========================================

        TrajectoryActionBuilder secondLaunchB =
                flowerBPickup.fresh()
                        .strafeTo(
                                new Vector2d(-60, -13.5)
                        )
                        //.waitSeconds(1)
                        .endTrajectory();

        // ========================================
        // NORMAL PARKING
        // ========================================

        TrajectoryActionBuilder parkPositionB =
                secondLaunchB.fresh()
                        .strafeToLinearHeading(
                                new Vector2d(-48, -65),
                                Math.toRadians(90)
                        )
                        .endTrajectory();

        TrajectoryActionBuilder backToFlowerLineupB = flowerBPickup.fresh()
                .strafeTo(new Vector2d(-50,-28))
                .endTrajectory();

        // ========================================
        // PARALLEL DRIVE / RAMP ACTIONS
        // ========================================

        ParallelAction driveAndRampDownB =
                new ParallelAction(
                        intake.rampsDownAction(),
                        flowerBLineUp.build()
                );

        ParallelAction driveAndRampUpB =
                new ParallelAction(
                        intake.rampsUpAction(),
                        secondLaunchB.build()
                );



        // ========================================
        // AUTONOMOUS ACTION SEQUENCE
        // ========================================

        List<Action> actions = new ArrayList<>();

        // Start launcher and intake.
        actions.add(launcher.startPollenLauncherAction());
        actions.add(intake.startIntakeAction());
        actions.add(launchPositionB.build());
        actions.add(waitForAllianceAprilTag(vision));
        actions.add(new SleepAction(.5));
        actions.add(launcher.launchFourPollenAction());
        actions.add(flowerBLineUp.build());
        actions.add(intake.rampsDownAction());
        actions.add(flowerBPickup.build());
        actions.add(intake.rampsShakeAction());
        actions.add(backToFlowerLineupB.build());
        actions.add(driveAndRampUpB);
        actions.add(waitForAllianceAprilTag(vision));
        actions.add(launcher.launchFourPollenAction());

        // ----------------------------------------
        // NORMAL PARKING
        // ----------------------------------------

        actions.add(parkPositionB.build());

        // ========================================
        // WAIT FOR START
        // ========================================

        try {

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            // Reset elapsed autonomous timer.
            resetRuntime();

            final boolean[] driveFinished = {false};

            // ========================================
            // NORMAL AUTONOMOUS SEQUENCE
            // ========================================

            Action driveSequence =
                    new SequentialAction(
                            new SequentialAction(actions),

                            packet -> {
                                driveFinished[0] = true;
                                return false;
                            }
                    );

            // ========================================
            // CONTINUOUS HEADING MONITOR
            // ========================================

            Action headingMonitor =
                    monitorHeadingContinuous(
                            drive,
                            driveFinished
                    );

            // ========================================
            // APPLY 25-SECOND FAILSAFE
            // ========================================

            Action protectedSequence =
                    parkAtTimeout(
                            drive,
                            launcher,
                            intake,
                            driveSequence,
                            new Vector2d(-48, -56),
                            90.0
                    );

            Action completedSequence =
                    new SequentialAction(
                            protectedSequence,

                            packet -> {
                                driveFinished[0] = true;
                                return false;
                            }
                    );

            // ========================================
            // EXECUTE AUTONOMOUS
            // ========================================

            Actions.runBlocking(
                    new ParallelAction(
                            completedSequence,
                            headingMonitor
                    )
            );

            // ========================================
            // FINAL ROBOT POSITION
            // ========================================

            drive.updatePoseEstimate();

            Pose2d finalPose =
                    drive.localizer.getPose();

            telemetry.addLine("=== AUTO COMPLETE ===");

            telemetry.addData(
                    "X",
                    finalPose.position.x
            );

            telemetry.addData(
                    "Y",
                    finalPose.position.y
            );

            telemetry.addData(
                    "Heading (deg)",
                    Math.toDegrees(
                            finalPose.heading.toDouble()
                    )
            );

            telemetry.update();

        } finally {

            // ========================================
            // CLEANUP FOR TELEOP TRANSITION
            // ========================================

            try {

                // Stop launcher and intake even if
                // autonomous ends unexpectedly.
                stopMechanisms(launcher, intake);

            } finally {

                // Release webcam for TeleOp.
                vision.close();
            }
        }
    }
}
