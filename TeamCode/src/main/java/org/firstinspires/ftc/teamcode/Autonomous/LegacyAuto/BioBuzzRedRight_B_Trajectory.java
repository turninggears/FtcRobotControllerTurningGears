package org.firstinspires.ftc.teamcode.Autonomous.LegacyAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDriveBiobuzz;
import org.firstinspires.ftc.teamcode.System.Intake;
import org.firstinspires.ftc.teamcode.System.Launcher;
import org.firstinspires.ftc.teamcode.System.MatchSettings;
import org.firstinspires.ftc.teamcode.System.Vision;

@Config
@Disabled
@Autonomous(name = "BioBuzz Red Right B - Trajectory", group = "Autonomous")
public class BioBuzzRedRight_B_Trajectory extends LinearOpMode {

    public static double AUTO_PARK_TRIGGER_SECONDS = 24.0;
    public static double EMERGENCY_PARK_TIMEOUT_SECONDS = 4.0;
    public static double SIDE_A_TAG_TIMEOUT_SECONDS = 1.0;
    public static int REQUIRED_TAG_DETECTIONS = 3;
    public static double GARDEN_WAIT_SECONDS = 0.5;
    public static double FLOWER_A_WAIT_SECONDS = 1.0;
    public static double FLOWER_B_WAIT_SECONDS = 1.0;

    public static double LAUNCH_A_X = 60, LAUNCH_A_Y = -10.5;
    public static double LAUNCH_A_RETURN_X = 60, LAUNCH_A_RETURN_Y = -10.5;
    public static double GARDEN_START_X = 65, GARDEN_START_Y = -55;
    public static double GARDEN_END_X = 60, GARDEN_END_Y = -55;
    public static double FLOWER_A_LINE_X = 28, FLOWER_A_LINE_Y = -42;
    public static double FLOWER_A_PICK_X = 28, FLOWER_A_PICK_Y = -52;
    public static double PUSH_WAYPOINT_1_X = 4, PUSH_WAYPOINT_1_Y = -56;
    public static double PARTNER_PARK_X = -5, PARTNER_PARK_Y = -56;
    public static double PUSH_WAYPOINT_2_X = 4, PUSH_WAYPOINT_2_Y = -36;
    public static double B_WAYPOINT_X = -38, B_WAYPOINT_Y = -36;
    public static double B_INTERMEDIATE_X = -30, B_INTERMEDIATE_Y = -36;
    public static double LAUNCH_B_X = -56, LAUNCH_B_Y = -13.5;
    public static double FLOWER_B_LINE_X = -50, FLOWER_B_LINE_Y = -28;
    public static double FLOWER_B_PICK_X = -61.5, FLOWER_B_PICK_Y = -28;
    public static double PARK_B_X = -48, PARK_B_Y = -65;

    private void stopMechanisms(Launcher launcher, Intake intake) {
        launcher.stopPollenLauncher();
        intake.stop();
    }

    private void stopDrive(MecanumDriveBiobuzz drive) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    private Action waitForTag(Vision vision, String label, double timeout, boolean[] result) {
        return new Action() {
            long start = -1;
            int consecutive = 0;
            @Override public boolean run(TelemetryPacket packet) {
                if (start < 0) { start = System.nanoTime(); result[0] = false; }
                boolean visible = vision.isAllianceClusterVisible();
                consecutive = visible ? consecutive + 1 : 0;
                double elapsed = (System.nanoTime() - start) / 1e9;
                packet.put(label + " Tag Visible", visible);
                packet.put(label + " Consecutive", consecutive);
                packet.put(label + " Wait Seconds", elapsed);
                if (consecutive >= Math.max(1, REQUIRED_TAG_DETECTIONS)) {
                    result[0] = true;
                    packet.put(label + " Result", "DETECTED");
                    return false;
                }
                if (timeout > 0 && elapsed >= timeout) {
                    packet.put(label + " Result", "TIMEOUT / SKIP");
                    return false;
                }
                return true;
            }
        };
    }

    /** Select a prebuilt branch only when reached; do not run both branches. */
    private Action choose(boolean[] condition, Action ifTrue, Action ifFalse) {
        return new Action() {
            Action selected;
            @Override public boolean run(TelemetryPacket packet) {
                if (selected == null) selected = condition[0] ? ifTrue : ifFalse;
                return selected.run(packet);
            }
        };
    }


    private Action parkAtTimeout(
            MecanumDriveBiobuzz drive,
            Launcher launcher,
            Intake intake,
            Action normalSequence) {

        return new Action() {

            Action emergencyParking = null;
            boolean triggered = false;
            long startNanos = -1;
            long parkStartNanos = -1;

            @Override
            public boolean run(TelemetryPacket packet) {

                if (startNanos < 0) {
                    startNanos = System.nanoTime();
                }

                double elapsed =
                        (System.nanoTime() - startNanos) / 1e9;

                packet.put("Auto Elapsed", elapsed);
                packet.put("Emergency Triggered", triggered);

                if (!triggered &&
                        elapsed >= AUTO_PARK_TRIGGER_SECONDS) {

                    triggered = true;
                    parkStartNanos = System.nanoTime();

                    stopMechanisms(launcher, intake);
                    stopDrive(drive);

                    drive.updatePoseEstimate();
                    Pose2d currentPose =
                            drive.localizer.getPose();

                    packet.put("Emergency Start X",
                            currentPose.position.x);
                    packet.put("Emergency Start Y",
                            currentPose.position.y);

                    emergencyParking =
                            drive.actionBuilder(currentPose)
                                    .strafeToLinearHeading(
                                            new Vector2d(
                                                    PARK_B_X,
                                                    PARK_B_Y),
                                            Math.toRadians(90))
                                    .build();
                }

                if (triggered) {

                    double parkElapsed =
                            (System.nanoTime() - parkStartNanos)
                                    / 1e9;

                    packet.put("Auto Phase", "EMERGENCY PARK");
                    packet.put("Park Elapsed", parkElapsed);

                    if (parkElapsed >=
                            EMERGENCY_PARK_TIMEOUT_SECONDS) {

                        stopDrive(drive);

                        packet.put("Auto Phase",
                                "PARK TIMEOUT - STOPPED");

                        return false;
                    }

                    boolean running =
                            emergencyParking.run(packet);

                    if (!running) {
                        stopDrive(drive);
                    }

                    return running;
                }

                packet.put("Auto Phase", "NORMAL AUTO");

                boolean running = normalSequence.run(packet);

                if (!running) {
                    stopDrive(drive);
                }

                return running;
            }
        };
    }

    private Action headingMonitor(MecanumDriveBiobuzz drive, boolean[] finished) {
        return packet -> {
            if (finished[0]) return false;
            // Read pose as updated by the active Road Runner trajectory.
            Pose2d pose = drive.localizer.getPose();
            double heading = (Math.toDegrees(pose.heading.toDouble()) % 360 + 360) % 360;
            packet.put("Pose X", pose.position.x);
            packet.put("Pose Y", pose.position.y);
            packet.put("Actual Heading", heading);
            return true;
        };
    }

    @Override public void runOpMode() {
        Pose2d startPose = new Pose2d(63.5, -10.5, Math.toRadians(0));
        MecanumDriveBiobuzz drive = new MecanumDriveBiobuzz(hardwareMap, startPose);
        Launcher launcher = new Launcher(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);
        MatchSettings.setAlliance(MatchSettings.Alliance.RED);
        Vision vision = new Vision(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Autonomous", "BioBuzz Red Right B - Trajectory");
        telemetry.addData("Alliance", "RED");
        telemetry.addLine("Waiting for START");
        telemetry.update();

        boolean[] sideATag = {false};
        boolean[] sideBTag = {false};
        boolean[] sideBReturnTag = {false};

        // A SIDE: each .fresh() starts at the planned endpoint of the preceding builder.
        TrajectoryActionBuilder launchA = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(LAUNCH_A_X, LAUNCH_A_Y), Math.toRadians(0))
                .waitSeconds(2.0).endTrajectory();
        TrajectoryActionBuilder gardenApproach = launchA.fresh()
                .strafeToLinearHeading(new Vector2d(65, -10.5), Math.toRadians(285))
                .strafeToLinearHeading(new Vector2d(GARDEN_START_X, GARDEN_START_Y), Math.toRadians(285))
                .strafeToLinearHeading(new Vector2d(GARDEN_END_X, GARDEN_END_Y), Math.toRadians(285))
                .waitSeconds(GARDEN_WAIT_SECONDS).endTrajectory();
        TrajectoryActionBuilder returnLaunchA = gardenApproach.fresh()
                .strafeToLinearHeading(new Vector2d(LAUNCH_A_RETURN_X, LAUNCH_A_RETURN_Y), Math.toRadians(0))
                .endTrajectory();

        // Optional A pickup, only when Side A AprilTag was detected.
        TrajectoryActionBuilder flowerALine = returnLaunchA.fresh()
                .strafeToLinearHeading(new Vector2d(FLOWER_A_LINE_X, FLOWER_A_LINE_Y), Math.toRadians(270))
                .endTrajectory();
        TrajectoryActionBuilder flowerAPick = flowerALine.fresh()
                .strafeToLinearHeading(new Vector2d(FLOWER_A_PICK_X, FLOWER_A_PICK_Y), Math.toRadians(270))
                .waitSeconds(FLOWER_A_WAIT_SECONDS).endTrajectory();
        TrajectoryActionBuilder flowerAReturn = flowerAPick.fresh()
                .strafeToLinearHeading(new Vector2d(FLOWER_A_LINE_X, FLOWER_A_LINE_Y), Math.toRadians(270))
                .endTrajectory();

        // Two alternative entry paths to the same first push waypoint.
        // The chosen path depends on whether Flower A pickup was executed.
        TrajectoryActionBuilder pushFromAFlower = flowerAReturn.fresh()
                .strafeToLinearHeading(new Vector2d(PUSH_WAYPOINT_1_X, PUSH_WAYPOINT_1_Y), Math.toRadians(0))
                .endTrajectory();
        TrajectoryActionBuilder pushFromALaunch = returnLaunchA.fresh()
                .strafeToLinearHeading(new Vector2d(PUSH_WAYPOINT_1_X, PUSH_WAYPOINT_1_Y), Math.toRadians(0))
                .endTrajectory();

        // Common partner push and crossing, beginning at push waypoint 1.
        Pose2d pushStartPose = new Pose2d(PUSH_WAYPOINT_1_X, PUSH_WAYPOINT_1_Y, Math.toRadians(0));
        TrajectoryActionBuilder partnerPark = drive.actionBuilder(pushStartPose)
                .strafeToLinearHeading(new Vector2d(PARTNER_PARK_X, PARTNER_PARK_Y), Math.toRadians(0))
                .endTrajectory();
        TrajectoryActionBuilder pushRetreat = partnerPark.fresh()
                .strafeToLinearHeading(new Vector2d(PUSH_WAYPOINT_1_X, PUSH_WAYPOINT_1_Y), Math.toRadians(0))
                .endTrajectory();
        TrajectoryActionBuilder pushWaypoint2 = pushRetreat.fresh()
                .strafeToLinearHeading(new Vector2d(PUSH_WAYPOINT_2_X, PUSH_WAYPOINT_2_Y), Math.toRadians(0))
                .endTrajectory();
        TrajectoryActionBuilder bWaypoint = pushWaypoint2.fresh()
                .strafeToLinearHeading(new Vector2d(B_WAYPOINT_X, B_WAYPOINT_Y), Math.toRadians(180))
                .endTrajectory();
        TrajectoryActionBuilder bIntermediate = bWaypoint.fresh()
                .strafeToLinearHeading(new Vector2d(B_INTERMEDIATE_X, B_INTERMEDIATE_Y), Math.toRadians(180))
                .endTrajectory();
        TrajectoryActionBuilder launchB = bIntermediate.fresh()
                .strafeToLinearHeading(new Vector2d(LAUNCH_B_X, LAUNCH_B_Y), Math.toRadians(180))
                .endTrajectory();

        // B SIDE: launch, lower ramps, collect flower, return, then launch again.
        TrajectoryActionBuilder flowerBLine = launchB.fresh()
                .strafeToLinearHeading(new Vector2d(FLOWER_B_LINE_X, FLOWER_B_LINE_Y), Math.toRadians(180))
                .waitSeconds(0.75).endTrajectory();
        TrajectoryActionBuilder flowerBPick = flowerBLine.fresh()
                .strafeToLinearHeading(new Vector2d(FLOWER_B_PICK_X, FLOWER_B_PICK_Y), Math.toRadians(180))
                .waitSeconds(FLOWER_B_WAIT_SECONDS).endTrajectory();
        TrajectoryActionBuilder flowerBReturn = flowerBPick.fresh()
                .strafeToLinearHeading(new Vector2d(FLOWER_B_LINE_X, FLOWER_B_LINE_Y), Math.toRadians(180))
                .endTrajectory();
        TrajectoryActionBuilder returnLaunchB = flowerBReturn.fresh()
                .strafeToLinearHeading(new Vector2d(LAUNCH_B_X, LAUNCH_B_Y), Math.toRadians(180))
                .endTrajectory();

        // Different normal park starting poses depending on whether B was visited.
        TrajectoryActionBuilder parkAfterB = returnLaunchB.fresh()
                .strafeToLinearHeading(new Vector2d(PARK_B_X, PARK_B_Y), Math.toRadians(90))
                .endTrajectory();
        TrajectoryActionBuilder parkWithoutBPickup = launchB.fresh()
                .strafeToLinearHeading(new Vector2d(PARK_B_X, PARK_B_Y), Math.toRadians(90))
                .endTrajectory();

        Action optionalA = new SequentialAction(
                launcher.launchFourPollenAction(),
                new ParallelAction(intake.rampsDownAction(), flowerALine.build()),
                flowerAPick.build(),
                flowerALine.build(),
                new ParallelAction(intake.rampsUpAction(), pushFromAFlower.build())
        );
        Action skipA = pushFromALaunch.build();

        Action sideBSequence = new SequentialAction(
                launcher.launchFourPollenAction(),
                new ParallelAction(intake.rampsDownAction(), flowerBLine.build()),
                flowerBPick.build(),
                intake.rampsShakeAction(),
                flowerBReturn.build(),
                new ParallelAction(intake.rampsUpAction(), returnLaunchB.build()),
                waitForTag(vision, "Side B Return", 0, sideBReturnTag),
                launcher.launchFourPollenAction(),
                parkAfterB.build()
        );

        Action normalSequence = new SequentialAction(
                launcher.startPollenLauncherAction(),
                intake.startIntakeAction(),
                launchA.build(),
                launcher.launchFourPollenAction(),
                gardenApproach.build(),
                returnLaunchA.build(),
                waitForTag(vision, "Side A", SIDE_A_TAG_TIMEOUT_SECONDS, sideATag),
                choose(sideATag, optionalA, skipA),
                partnerPark.build(),
                pushRetreat.build(),
                pushWaypoint2.build(),
                bWaypoint.build(),
                bIntermediate.build(),
                launchB.build(),
                waitForTag(vision, "Side B", 0, sideBTag),
                choose(sideBTag, sideBSequence, parkWithoutBPickup.build())
        );

        try {
            waitForStart();
            if (isStopRequested()) return;
            resetRuntime();
            boolean[] finished = {false};
            Action protectedSequence = parkAtTimeout(drive, launcher, intake, normalSequence);
            Action completion = new SequentialAction(protectedSequence, packet -> {
                finished[0] = true;
                return false;
            });
            Actions.runBlocking(new ParallelAction(completion, headingMonitor(drive, finished)));
            drive.updatePoseEstimate();
            Pose2d finalPose = drive.localizer.getPose();
            telemetry.addLine("=== AUTO COMPLETE ===");
            telemetry.addData("X", finalPose.position.x);
            telemetry.addData("Y", finalPose.position.y);
            telemetry.addData("Heading (deg)", Math.toDegrees(finalPose.heading.toDouble()));
            telemetry.update();
        } finally {
            stopDrive(drive);
            try { stopMechanisms(launcher, intake); }
            finally { vision.close(); }
        }
    }
}
