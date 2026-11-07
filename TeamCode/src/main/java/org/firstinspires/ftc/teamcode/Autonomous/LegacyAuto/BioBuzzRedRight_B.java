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
@Autonomous(name = "BioBuzz Red Right B", group = "Autonomous")
public class BioBuzzRedRight_B extends LinearOpMode {

    public static double AUTO_PARK_TRIGGER_SECONDS = 24.0;
    public static double SIDE_A_TAG_TIMEOUT_SECONDS = 1.0;
    public static int REQUIRED_TAG_DETECTIONS = 3;
    public static double GARDEN_WAIT_SECONDS = 0.5;
    public static double FLOWER_A_WAIT_SECONDS = 1.0;
    public static double FLOWER_B_WAIT_SECONDS = 1.0;

    // Field coordinates, inches. Headings are degrees.
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
    public static double LAUNCH_B_X = -60, LAUNCH_B_Y = -13.5;
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

    /** Builds a fresh Road Runner trajectory only when this action starts. */
    private Action driveTo(MecanumDriveBiobuzz drive,
                           double x, double y, double headingDegrees) {
        return new Action() {
            private Action trajectory;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (trajectory == null) {
                    drive.updatePoseEstimate();
                    Pose2d currentPose = drive.localizer.getPose();
                    trajectory = drive.actionBuilder(currentPose)
                            .strafeToLinearHeading(
                                    new Vector2d(x, y),
                                    Math.toRadians(headingDegrees))
                            .build();
                }
                return trajectory.run(packet);
            }
        };
    }

    /** A short Road Runner-compatible wait that starts when reached. */
    private Action waitSeconds(double seconds) {
        return new Action() {
            private long startNanos = -1;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (startNanos < 0) startNanos = System.nanoTime();
                return (System.nanoTime() - startNanos) / 1e9 < seconds;
            }
        };
    }

    /**
     * Waits for consecutive alliance-tag detections. A timeout of zero
     * means wait until detected or the global parking trigger interrupts.
     *
     * IMPORTANT: isAllianceClusterVisible() is alliance-wide; it does not
     * prove the detected tag belongs to the intended A or B side.
     */
    private Action checkAprilTag(Vision vision, String label,
                                 double timeoutSeconds, boolean[] detected) {
        return new Action() {
            private long startNanos = -1;
            private int consecutive = 0;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (startNanos < 0) {
                    startNanos = System.nanoTime();
                    detected[0] = false;
                }

                boolean visible = vision.isAllianceClusterVisible();
                consecutive = visible ? consecutive + 1 : 0;
                double elapsed = (System.nanoTime() - startNanos) / 1e9;

                packet.put(label + " Tag Visible", visible);
                packet.put(label + " Tag Consecutive", consecutive);
                packet.put(label + " Tag Wait Seconds", elapsed);

                if (consecutive >= Math.max(1, REQUIRED_TAG_DETECTIONS)) {
                    detected[0] = true;
                    packet.put(label + " Tag Result", "DETECTED");
                    return false;
                }
                if (timeoutSeconds > 0 && elapsed >= timeoutSeconds) {
                    packet.put(label + " Tag Result", "TIMEOUT - SKIP");
                    return false;
                }
                packet.put(label + " Tag Result", "WAITING");
                return true;
            }
        };
    }

    /** Chooses an entire branch when the preceding tag check finishes. */
    private Action ifDetected(boolean[] detected, Action yesAction) {
        return new Action() {
            @Override
            public boolean run(TelemetryPacket packet) {
                if (!detected[0]) return false;
                return yesAction.run(packet);
            }
        };
    }

    /** Always monitors the 25-second limit, even during AprilTag waits. */
    private Action parkAtTimeout(MecanumDriveBiobuzz drive,
                                 Launcher launcher, Intake intake,
                                 Action normalSequence) {
        return new Action() {
            private Action emergencyPark;
            private boolean parkingStarted;

            @Override
            public boolean run(TelemetryPacket packet) {
                double elapsed = getRuntime();
                packet.put("Auto Elapsed Seconds", elapsed);
                packet.put("Emergency Parking", parkingStarted);

                if (!parkingStarted && elapsed >= AUTO_PARK_TRIGGER_SECONDS) {
                    parkingStarted = true;
                    stopMechanisms(launcher, intake);
                    stopDrive(drive);
                    // Read Pinpoint again after interrupting the active path.
                    drive.updatePoseEstimate();
                    Pose2d currentPose = drive.localizer.getPose();
                    emergencyPark = drive.actionBuilder(currentPose)
                            .strafeToLinearHeading(
                                    new Vector2d(PARK_B_X, PARK_B_Y),
                                    Math.toRadians(90))
                            .build();
                }

                if (parkingStarted) {
                    packet.put("Auto Phase", "EMERGENCY PARK B");
                    return emergencyPark.run(packet);
                }

                boolean running = normalSequence.run(packet);
                if (!running) {
                    stopDrive(drive);
                    packet.put("Auto Phase", "COMPLETE");
                }
                return running;
            }
        };
    }

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(63.5, -10.5, Math.toRadians(0));
        MecanumDriveBiobuzz drive = new MecanumDriveBiobuzz(hardwareMap, startPose);
        Launcher launcher = new Launcher(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);

        MatchSettings.setAlliance(MatchSettings.Alliance.RED);
        Vision vision = new Vision(hardwareMap);

        telemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Autonomous", "BioBuzz Red Right B");
        telemetry.addData("Alliance", "RED");
        telemetry.addLine("Waiting for START");
        telemetry.update();

        boolean[] sideATag = {false};
        boolean[] sideBTag = {false};
        boolean[] sideBReturnTag = {false};

        // First launch, garden pickup, then Launch A #2.
        Action opening = new SequentialAction(
                launcher.startPollenLauncherAction(),
                intake.startIntakeAction(),
                driveTo(drive, LAUNCH_A_X, LAUNCH_A_Y, 0),
                waitSeconds(2.0),
                launcher.launchFourPollenAction(),
                driveTo(drive, 65, -10.5, 285),
                driveTo(drive, GARDEN_START_X, GARDEN_START_Y, 285),
                driveTo(drive, GARDEN_END_X, GARDEN_END_Y, 285),
                waitSeconds(GARDEN_WAIT_SECONDS),
                driveTo(drive, LAUNCH_A_RETURN_X, LAUNCH_A_RETURN_Y, 0),
                checkAprilTag(vision, "Side A", SIDE_A_TAG_TIMEOUT_SECONDS, sideATag)
        );

// Only execute this if Launch A sees its required tag.
        Action sideAOptional = new SequentialAction(
                launcher.launchFourPollenAction(),

                // Move to Flower A lineup while lowering ramps
                new ParallelAction(
                        intake.rampsDownAction(),
                        driveTo(drive, FLOWER_A_LINE_X, FLOWER_A_LINE_Y, 270)),

                // Move to Flower A pickup position
                driveTo(drive, FLOWER_A_PICK_X, FLOWER_A_PICK_Y, 270),

                // Wait at pickup position
                waitSeconds(FLOWER_A_WAIT_SECONDS),

                // Return to Flower A lineup

                driveTo(drive, FLOWER_A_LINE_X, FLOWER_A_LINE_Y, 270),
                intake.rampsUpAction()
        );

        // Push partner, retreat, then cross to the B side.
        Action partnerPushAndCross = new SequentialAction(
                driveTo(drive, PUSH_WAYPOINT_1_X, PUSH_WAYPOINT_1_Y, 0),
                driveTo(drive, PARTNER_PARK_X, PARTNER_PARK_Y, 0),
                driveTo(drive, PUSH_WAYPOINT_1_X, PUSH_WAYPOINT_1_Y, 0),
                driveTo(drive, PUSH_WAYPOINT_2_X, PUSH_WAYPOINT_2_Y, 0),
                driveTo(drive, B_WAYPOINT_X, B_WAYPOINT_Y, 180),
                driveTo(drive, -50,-40,180),
                driveTo(drive, LAUNCH_B_X, LAUNCH_B_Y, 180),
                checkAprilTag(vision, "Side B", 0, sideBTag)
        );

        // Side B: launch, collect Flower B, return and launch again.
        Action sideBSequence = new SequentialAction(
                launcher.launchFourPollenAction(),
                new ParallelAction(
                        intake.rampsDownAction(),
                        driveTo(drive, FLOWER_B_LINE_X, FLOWER_B_LINE_Y, 180)),
                waitSeconds(0.75),
                driveTo(drive, FLOWER_B_PICK_X, FLOWER_B_PICK_Y, 180),
                waitSeconds(FLOWER_B_WAIT_SECONDS),
                intake.rampsShakeAction(),
                driveTo(drive, FLOWER_B_LINE_X, FLOWER_B_LINE_Y, 180),
                new ParallelAction(
                        intake.rampsUpAction(),
                        driveTo(drive, LAUNCH_B_X, LAUNCH_B_Y, 180)),
                checkAprilTag(vision, "Side B Return", 0, sideBReturnTag),
                launcher.launchFourPollenAction()
        );

        // Normal finish also uses measured position, not a .fresh() chain.
        Action normalSequence = new SequentialAction(
                opening,
                ifDetected(sideATag, sideAOptional),
                partnerPushAndCross,
                ifDetected(sideBTag, sideBSequence),
                driveTo(drive, PARK_B_X, PARK_B_Y, 90)
        );

        try {
            waitForStart();
            if (isStopRequested()) return;
            resetRuntime();

            Actions.runBlocking(parkAtTimeout(drive, launcher, intake, normalSequence));

            drive.updatePoseEstimate();
            Pose2d finalPose = drive.localizer.getPose();
            telemetry.addLine("=== AUTO COMPLETE ===");
            telemetry.addData("X", finalPose.position.x);
            telemetry.addData("Y", finalPose.position.y);
            telemetry.addData("Heading (deg)",
                    Math.toDegrees(finalPose.heading.toDouble()));
            telemetry.update();
        } finally {
            stopDrive(drive);
            try {
                stopMechanisms(launcher, intake);
            } finally {
                vision.close();
            }
        }
    }
}
