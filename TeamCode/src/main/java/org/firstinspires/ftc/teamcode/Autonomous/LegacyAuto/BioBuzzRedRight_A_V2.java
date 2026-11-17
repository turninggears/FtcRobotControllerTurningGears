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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDriveBiobuzz;
import org.firstinspires.ftc.teamcode.System.Intake;
import org.firstinspires.ftc.teamcode.System.Launcher;
import org.firstinspires.ftc.teamcode.System.MatchSettings;
import org.firstinspires.ftc.teamcode.System.Vision;

@Config
@Autonomous(name = "BioBuzz Red Right A V2", group = "Autonomous")
public class BioBuzzRedRight_A_V2 extends LinearOpMode {

    public static double AUTO_PARK_TRIGGER_SECONDS = 27.0;
    public static double SIDE_A_TAG_TIMEOUT_SECONDS = .01;
    public static int REQUIRED_TAG_DETECTIONS = 3;
    public static double GARDEN_WAIT_SECONDS = 0.01;
    public static double FLOWER_A_WAIT_SECONDS = .5;//was.75
    public static double FLOWER_B_WAIT_SECONDS = .75;

    // Field coordinates, inches. Headings are degrees.
    public static double LAUNCH_A_X = 60, LAUNCH_A_Y = -35;
    public static double LAUNCH_A_TAG_READ_X = 60, LAUNCH_A_TAG_READ_Y = -35;
    public static double GARDEN_START_X = 70, GARDEN_START_Y = -50;
    public static double GARDEN_END_X = 68, GARDEN_END_Y = -61;
    public static double FLOWER_A_LINE_X = 24, FLOWER_A_LINE_Y = -50;
    public static double FLOWER_A_PICK_X = 24, FLOWER_A_PICK_Y = -62;
     public static double B_WAYPOINT_X = -15, B_WAYPOINT_Y = -35;
    public static double LAUNCH_B_X = -54, LAUNCH_B_Y = -12, LAUNCH_B_HEADING = 185;
    public static double FLOWER_B_LINE_X = -50, FLOWER_B_LINE_Y = -25;
    public static double FLOWER_B_PICK_X = -58, FLOWER_B_PICK_Y = -20;
    public static double PARK_A_X = -20, PARK_A_Y = -52;

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
                boolean running = trajectory.run(packet);

                drive.updatePoseEstimate();
                Pose2d actualPose = drive.localizer.getPose();

                packet.put("Drive Target X", x);
                packet.put("Drive Target Y", y);
                packet.put("Drive Actual X", actualPose.position.x);
                packet.put("Drive Actual Y", actualPose.position.y);
                packet.put("Drive Heading",
                        Math.toDegrees(actualPose.heading.toDouble()));

                return running;
            }
        };
    }
    private Action turnTo(MecanumDriveBiobuzz drive, double headingDegrees) {
        return new Action() {
            private Action turnAction;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (turnAction == null) {
                    drive.updatePoseEstimate();

                    Pose2d currentPose = drive.localizer.getPose();

                    turnAction = drive.actionBuilder(currentPose)
                            .turnTo(Math.toRadians(headingDegrees))
                            .build();
                }

                return turnAction.run(packet);
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
                launcher.updatePollenLauncher();
                double elapsed = getRuntime();

                packet.put("Auto Elapsed Seconds", elapsed);
                packet.put("Emergency Parking", parkingStarted);

// Launcher telemetry
                packet.put("Launcher Target Velocity", Launcher.LAUNCHER_VELOCITY);
                packet.put("Launcher Actual Velocity", launcher.getPollenLauncherVelocity());

                packet.put("Launcher Velocity Error",
                        Launcher.LAUNCHER_VELOCITY - launcher.getPollenLauncherVelocity());

                if (!parkingStarted && elapsed >= AUTO_PARK_TRIGGER_SECONDS) {
                    parkingStarted = true;

                    // Keep intake and launcher running during emergency parking.
                    // Stop only the current driving movement.
                    stopDrive(drive);

                    // Read current Pinpoint position.
                    drive.updatePoseEstimate();
                    Pose2d currentPose = drive.localizer.getPose();

                    // Drive to park, then raise ramps and stop mechanisms.
                    emergencyPark = new SequentialAction(

                            drive.actionBuilder(currentPose)
                                    .strafeToLinearHeading(
                                            new Vector2d(PARK_A_X, PARK_A_Y),
                                            Math.toRadians(90))
                                    .build(),

                            // Raise ramps only after reaching parking position
                            intake.rampsUpAction(),

                            // Stop intake and launcher after ramps are commanded UP
                            packet2 -> {
                                stopMechanisms(launcher, intake);
                                return false;
                            }
                    );
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

    Action noAction = packet -> false;

    private Action checkIfHiveUpAction (Vision vision, boolean[] tag, Action ifTrue, Action ifFalse) {
        return vision.chooseAction(tag[0], ifTrue, ifFalse);
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
        telemetry.addData("Autonomous", "BioBuzz Red Right A v2");
        telemetry.addData("Alliance", "Red");
        telemetry.addLine("Waiting for START");
        telemetry.update();

        boolean[] sideATag = {false};
        boolean[] sideBTag = {false};
        boolean[] sideBReturnTag = {false};

        // First launch, garden pickup, then Launch A #2.
        Action opening = new SequentialAction(
                launcher.startPollenLauncherAction(),
                intake.startIntakeAction(),
                driveTo(drive, LAUNCH_A_X, LAUNCH_A_Y, 330),
                waitSeconds(1.5),
                launcher.launchFourPollenAction(),
                driveTo(drive, GARDEN_END_X, GARDEN_END_Y, 270
                ),
                waitSeconds(GARDEN_WAIT_SECONDS),
                driveTo(drive, LAUNCH_A_X, LAUNCH_A_Y, 330),
                checkAprilTag(vision, "Side A", 0.0, sideATag),
                launcher.launchFourPollenAction(),
                new ParallelAction(
                        intake.rampsDownAction(),
                        driveTo(drive, FLOWER_A_LINE_X,FLOWER_A_LINE_Y,270)),
                driveTo(drive,FLOWER_A_PICK_X,FLOWER_A_PICK_Y,270),
                waitSeconds(1.25),
                driveTo(drive, FLOWER_A_LINE_X,FLOWER_A_LINE_Y,270),
                new ParallelAction(
                        intake.rampsUpAction(),
                        driveTo(drive,LAUNCH_A_X,LAUNCH_A_Y,325)

                ),
                checkAprilTag(vision, "Side A", 0.0, sideATag),
                 launcher.launchFourPollenAction()


        );

        Action endOptionalSequence = new SequentialAction(
                driveTo(drive,LAUNCH_A_TAG_READ_X,LAUNCH_A_TAG_READ_Y,315)
        );

        // Normal finish also uses measured position, not a .fresh() chain.
        Action normalSequence = new SequentialAction(
                opening,
                driveTo(drive, PARK_A_X, PARK_A_Y, 90),
                intake.rampsUpAction(),
                packet -> {
                    stopMechanisms(launcher, intake);
                    return false;
                }
        );

        try {
            waitForStart();
            if (isStopRequested()) return;
            resetRuntime();

            Actions.runBlocking(parkAtTimeout(drive, launcher, intake, normalSequence));

            drive.updatePoseEstimate();
            Pose2d finalPose = drive.localizer.getPose();
                           Math.toDegrees(finalPose.heading.toDouble());
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
