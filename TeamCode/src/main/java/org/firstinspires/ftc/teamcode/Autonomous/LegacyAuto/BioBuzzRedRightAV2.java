
package org.firstinspires.ftc.teamcode.Autonomous.LegacyAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
@Autonomous(name = "BioBuzz Red Right A V2", group = "Autonomous")
public class BioBuzzRedRightAV2 extends LinearOpMode {

    public static double AUTO_PARK_TRIGGER_SECONDS = 25.0;
    public static int REQUIRED_TAG_DETECTIONS = 3;

    // ========================================
    // STOP MECHANISMS
    // ========================================

    private void stopMechanisms(Launcher launcher, Intake intake) {
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
            Vector2d waypointPosition,
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

                if (!parkingStarted
                        && elapsed >= AUTO_PARK_TRIGGER_SECONDS) {

                    parkingStarted = true;

                    // Stop launcher and intake before parking.
                    stopMechanisms(launcher, intake);

                    // Update current robot position.
                    drive.updatePoseEstimate();

                    Pose2d currentPose =
                            drive.localizer.getPose();

                    // Current position -> Waypoint A -> Park A.
                    emergencyParking =
                            drive.actionBuilder(currentPose)
                                    .strafeTo(waypointPosition)
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

                if (parkingStarted) {

                    packet.put(
                            "Emergency Parking Status",
                            "PARKING"
                    );

                    return emergencyParking.run(packet);
                }

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

                if (consecutiveDetections >= REQUIRED_TAG_DETECTIONS) {

                    packet.put("AprilTag Status", "DETECTED");
                    return false;
                }

                packet.put("AprilTag Status", "WAITING");

                // Wait until detection or global failsafe.
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

            Pose2d actualPose = drive.localizer.getPose();

            double actualHeading =
                    Math.toDegrees(actualPose.heading.toDouble());

            actualHeading =
                    (actualHeading % 360 + 360) % 360;

            telemetry.addData(
                    "Actual Heading (deg)",
                    "%.1f",
                    actualHeading
            );

            telemetry.update();

            packet.put("Actual Heading (deg)", actualHeading);

            return true;
        };
    }

    // ========================================
    // MAIN AUTONOMOUS
    // ========================================

    @Override
    public void runOpMode() {

        Pose2d startPose =
                new Pose2d(63.5, -10.5, Math.toRadians(0));

        MecanumDriveBiobuzz drive =
                new MecanumDriveBiobuzz(hardwareMap, startPose);

        Launcher launcher =
                new Launcher(hardwareMap, telemetry);

        Intake intake =
                new Intake(hardwareMap, telemetry);

        // Select RED alliance before initializing Vision.
        MatchSettings.setAlliance(MatchSettings.Alliance.RED);

        Vision vision = new Vision(hardwareMap);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        telemetry.addData("Alliance", "RED");
        telemetry.addData("Autonomous", "Red Right A V2");
        telemetry.addLine("Waiting for START");
        telemetry.update();

        // ========================================
        // TRAJECTORIES
        // ========================================

        TrajectoryActionBuilder launchPositionA =
                drive.actionBuilder(startPose)
                        .setTangent(Math.toRadians(0))
                        .strafeTo(new Vector2d(60, -10.5))
                        .waitSeconds(2)
                        .endTrajectory();

        TrajectoryActionBuilder backToWall =
                launchPositionA.fresh()
                        .turnTo(Math.toRadians(285))
                        .strafeTo(new Vector2d(65, -10.5))
                        .endTrajectory();

        TrajectoryActionBuilder gardenPosition =
                backToWall.fresh()
                        .setTangent(Math.toRadians(270))
                        .strafeTo(new Vector2d(65, -55))
                        .endTrajectory();

        TrajectoryActionBuilder gardenShiftPosition =
                gardenPosition.fresh()
                        .strafeTo(new Vector2d(60, -55))
                        .endTrajectory();

        TrajectoryActionBuilder secondLaunchA =
                gardenShiftPosition.fresh()
                        .strafeToLinearHeading(
                                new Vector2d(60, -13.5),
                                Math.toRadians(0)
                        )
                        .waitSeconds(1)
                        .endTrajectory();

        TrajectoryActionBuilder flowerALineUp =
                secondLaunchA.fresh()
                        .strafeToLinearHeading(
                                new Vector2d(28, -42),
                                Math.toRadians(270)
                        )
                        .endTrajectory();

        TrajectoryActionBuilder flowerAPickup =
                flowerALineUp.fresh()
                        .setTangent(Math.toRadians(270))
                        .strafeTo(new Vector2d(28, -52))
                        .waitSeconds(1)
                        .endTrajectory();

        TrajectoryActionBuilder thirdLaunchA =
                flowerAPickup.fresh()
                        .strafeToLinearHeading(
                                new Vector2d(60, -10.5),
                                Math.toRadians(0)
                        )
                        .waitSeconds(1)
                        .endTrajectory();

        TrajectoryActionBuilder waypointA =
                thirdLaunchA.fresh()
                        .strafeTo(new Vector2d(32, -42))
                        .endTrajectory();

        TrajectoryActionBuilder parkPositionA =
                waypointA.fresh()
                        .strafeToLinearHeading(
                                new Vector2d(-23, -56),
                                Math.toRadians(90)
                        )
                        .endTrajectory();

        // ========================================
        // PARALLEL RAMP ACTIONS
        // ========================================

        ParallelAction driveAndRampDown =
                new ParallelAction(
                        intake.rampsDownAction(),
                        flowerALineUp.build()
                );

        ParallelAction driveAndRampUp =
                new ParallelAction(
                        intake.rampsUpAction(),
                        thirdLaunchA.build()
                );

        // ========================================
        // AUTONOMOUS ACTION SEQUENCE
        // ========================================

        List<Action> actions = new ArrayList<>();

        actions.add(launcher.startPollenLauncherAction());
        actions.add(intake.startIntakeAction());

        // Launch 1: No AprilTag check.
        actions.add(launchPositionA.build());
        actions.add(launcher.launchFourPollenAction());

        // Garden movement.
        actions.add(backToWall.build());
        actions.add(gardenPosition.build());
        actions.add(gardenShiftPosition.build());

        // Launch 2: Wait for RED alliance AprilTag.
        actions.add(secondLaunchA.build());
        actions.add(waitForAllianceAprilTag(vision));
        actions.add(launcher.launchFourPollenAction());

        // Flower pickup.
        actions.add(driveAndRampDown);
        actions.add(flowerAPickup.build());
        actions.add(driveAndRampUp);

        // Launch 3: No AprilTag check.
        actions.add(launcher.launchFourPollenAction());

        // Normal parking through Waypoint A.
        actions.add(waypointA.build());
        actions.add(parkPositionA.build());

        // ========================================
        // WAIT FOR START
        // ========================================

        try {

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            resetRuntime();

            final boolean[] driveFinished = {false};

            Action driveSequence =
                    new SequentialAction(
                            new SequentialAction(actions),

                            packet -> {
                                driveFinished[0] = true;
                                return false;
                            }
                    );

            Action headingMonitor =
                    monitorHeadingContinuous(
                            drive,
                            driveFinished
                    );

            // ========================================
            // 25-SECOND FAILSAFE
            // ========================================

            Action protectedSequence =
                    parkAtTimeout(
                            drive,
                            launcher,
                            intake,
                            driveSequence,
                            new Vector2d(32, -42),
                            new Vector2d(-23, -56),
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
            // FINAL TELEMETRY
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
                    Math.toDegrees(finalPose.heading.toDouble())
            );

            telemetry.update();

        } finally {

            // Stop mechanisms regardless of how
            // autonomous finishes.
            try {
                stopMechanisms(launcher, intake);
            } finally {
                // Release webcam for TeleOp.
                vision.close();
            }
        }
    }
}
