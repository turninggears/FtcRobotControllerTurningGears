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
@Autonomous(name = "BioBuzz Red Left Wait", group = "Autonomous")
public class BioBuzzRedLeft_Wait extends LinearOpMode {

    // ========================================
    // AUTONOMOUS SETTINGS
    // ========================================
    public static double AUTO_PARK_TRIGGER_SECONDS = 24.0;

    // AprilTag timeouts (seconds)
    private static final double FIRST_TAG_TIMEOUT = 10.0;
    private static final double SECOND_TAG_TIMEOUT = 3.0;

    // Three consecutive detections required
    public static int REQUIRED_TAG_DETECTIONS = 3;

    // Alternate Launch A
    private static final double LAUNCH_A_X = 60.0;
    private static final double LAUNCH_A_Y = -10.5;
    private static final double LAUNCH_A_HEADING = 0.0;

    // Park Alternate waypoint
    private static final double PARK_ALT_WAYPOINT_X = -48.0;
    private static final double PARK_ALT_WAYPOINT_Y = -36.0;

    // Final parking location
    private static final double PARK_X = -48.0;
    private static final double PARK_Y = -65.0;
    private static final double PARK_HEADING = 90.0;

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
    // 24-SECOND EMERGENCY PARKING
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

                // Interrupt normal autonomous at 24 seconds.
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

                // Continue normal autonomous before 24 seconds.
                return normalSequence.run(packet);
            }
        };
    }

    // ========================================
    // WAIT FOR RED ALLIANCE APRILTAG
    // ========================================

    private Action waitForAllianceAprilTag(
            Vision vision,
            double timeoutSeconds,
            boolean[] detected
    ) {

        return new Action() {

            private long startTime = 0;
            private int consecutiveDetections = 0;

            @Override
            public boolean run(TelemetryPacket packet) {

                if (startTime == 0) {
                    startTime = System.nanoTime();
                    detected[0] = false;
                }

                double elapsed =
                        (System.nanoTime() - startTime) / 1e9;

                boolean visible =
                        vision.isAllianceClusterVisible();



                if (visible) {
                    consecutiveDetections++;
                } else {
                    consecutiveDetections = 0;
                }

                packet.put("AprilTag Visible", visible);
                packet.put("Tag Wait Elapsed", elapsed);
                packet.put("Consecutive Detections",
                        consecutiveDetections);

                if (consecutiveDetections >= REQUIRED_TAG_DETECTIONS) {

                    detected[0] = true;
                    packet.put("AprilTag Status", "DETECTED");
                    return false;
                }

                if (timeoutSeconds >= 0
                        && elapsed >= timeoutSeconds) {

                    detected[0] = false;
                    packet.put("AprilTag Status", "TIMEOUT");
                    return false;
                }

                packet.put("AprilTag Status", "WAITING");

                return true;
            }
        };
    }

    // ========================================
// DRIVE FROM CURRENT POSITION
// ========================================

    private Action driveToCurrentPosition(
            MecanumDriveBiobuzz drive,
            double x,
            double y,
            Double headingDegrees,
            Double tangentDegrees
    ) {
        return new Action() {

            private Action movement = null;

            @Override
            public boolean run(TelemetryPacket packet) {

                if (movement == null) {

                    drive.updatePoseEstimate();

                    Pose2d currentPose =
                            drive.localizer.getPose();

                    TrajectoryActionBuilder builder =
                            drive.actionBuilder(currentPose);

                    if (tangentDegrees != null) {
                        builder = builder.setTangent(
                                Math.toRadians(tangentDegrees)
                        );
                    }

                    if (headingDegrees == null) {
                        builder = builder.strafeTo(
                                new Vector2d(x, y)
                        );
                    } else {
                        builder = builder.strafeToLinearHeading(
                                new Vector2d(x, y),
                                Math.toRadians(headingDegrees)
                        );
                    }

                    movement = builder.build();
                }

                return movement.run(packet);
            }
        };
    }

    private Action driveToCurrentPosition(
            MecanumDriveBiobuzz drive,
            double x,
            double y,
            Double headingDegrees
    ) {
        return driveToCurrentPosition(
                drive,
                x,
                y,
                headingDegrees,
                null
        );
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
        telemetry.addData("Autonomous", "Red Left Wait");
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
        // AUTONOMOUS ACTION SEQUENCE
        // ========================================

        // Each AprilTag check stores its result separately.
        final boolean[] firstTagB = {false};
        final boolean[] firstTagA = {false};
        final boolean[] secondTagB = {false};
        final boolean[] secondTagA = {false};

        // Path B: original launch and Flower B retrieval.
        // Preserve the original pickup waits and ramp shake.
        Action flowerBRetrieval = new SequentialAction(
                new ParallelAction(
                        intake.rampsDownAction(),
                        driveToCurrentPosition(drive, -50, -28, 180.0)
                ),
                new com.acmerobotics.roadrunner.SleepAction(0.75),
                driveToCurrentPosition(drive, -61.5, -28, 180.0,180.0),
                new com.acmerobotics.roadrunner.SleepAction(1.0),
                intake.rampsShakeAction(),
                driveToCurrentPosition(drive, -50, -28, 180.0),
                new ParallelAction(
                        intake.rampsUpAction(),
                        driveToCurrentPosition(drive, -60, -13.5, 180.0)
                )
        );

        // Path A: alternate launch and Flower A retrieval.
        // Coordinates and pickup wait come from BioBuzzRedRight_A.
        Action flowerARetrieval = new SequentialAction(
                new ParallelAction(
                        intake.rampsDownAction(),
                        driveToCurrentPosition(drive, 28, -42, 270.0)
                ),
                driveToCurrentPosition(drive, 28, -52, 270.0),
                new com.acmerobotics.roadrunner.SleepAction(1.0),
                driveToCurrentPosition(drive, 28, -42, 270.0),
                new ParallelAction(
                        intake.rampsUpAction(),
                        driveToCurrentPosition(drive, LAUNCH_A_X, LAUNCH_A_Y, LAUNCH_A_HEADING)
                ),
                new com.acmerobotics.roadrunner.SleepAction(1.0)
        );

        // Path B always parks directly after the second tag check.
        Action finishPathB = new SequentialAction(
                launcher.launchFourPollenAction(),
                flowerBRetrieval,
                waitForAllianceAprilTag(vision, SECOND_TAG_TIMEOUT, secondTagB),
                choose(
                        secondTagB,
                        new SequentialAction(
                                launcher.launchFourPollenAction(),
                                driveToCurrentPosition(drive, PARK_X, PARK_Y, PARK_HEADING)
                        ),
                        driveToCurrentPosition(drive, PARK_X, PARK_Y, PARK_HEADING)
                )
        );

        // Path A uses Park Alternate only if the second tag is missed.
        Action parkAlternate = new SequentialAction(
                driveToCurrentPosition(drive, 28, -42, 270.0),
                driveToCurrentPosition(
                        drive, PARK_ALT_WAYPOINT_X, PARK_ALT_WAYPOINT_Y, null
                ),
                driveToCurrentPosition(drive, PARK_X, PARK_Y, PARK_HEADING)
        );

        Action finishPathA = new SequentialAction(
                // No individual timeout: 24-second failsafe interrupts this wait.
                waitForAllianceAprilTag(vision, -1.0, firstTagA),
                launcher.launchFourPollenAction(),
                flowerARetrieval,
                waitForAllianceAprilTag(vision, SECOND_TAG_TIMEOUT, secondTagA),
                choose(
                        secondTagA,
                        new SequentialAction(
                                launcher.launchFourPollenAction(),
                                driveToCurrentPosition(drive, PARK_X, PARK_Y, PARK_HEADING)
                        ),
                        parkAlternate
                )
        );

        Action alternatePathA = new SequentialAction(
                // Cross to the opposite launch side and rotate to heading 0.
                driveToCurrentPosition(
                        drive, LAUNCH_A_X, LAUNCH_A_Y, LAUNCH_A_HEADING
                ),
                finishPathA
        );

        List<Action> actions = new ArrayList<>();
        actions.add(launcher.startPollenLauncherAction());
        actions.add(intake.startIntakeAction());
        actions.add(launchPositionB.build());
        actions.add(waitForAllianceAprilTag(
                vision, FIRST_TAG_TIMEOUT, firstTagB
        ));
        actions.add(choose(firstTagB, finishPathB, alternatePathA));

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
            // APPLY 24-SECOND FAILSAFE
            // ========================================

            Action protectedSequence =
                    parkAtTimeout(
                            drive,
                            launcher,
                            intake,
                            driveSequence,
                            new Vector2d(PARK_X, PARK_Y),
                            PARK_HEADING
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

    private Action choose(boolean[] condition, Action ifTrue, Action ifFalse) {
        return new Action() {
            Action selected;
            @Override public boolean run(TelemetryPacket packet) {
                if (selected == null) selected = condition[0] ? ifTrue : ifFalse;
                return selected.run(packet);
            }
        };
    }
}

