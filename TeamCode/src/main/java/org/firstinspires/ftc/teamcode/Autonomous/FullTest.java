package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.System.Intake;
import org.firstinspires.ftc.teamcode.System.Launcher;

@Config
@Autonomous(name = "FullTest", group = "Autonomous")
public class FullTest extends LinearOpMode {

    // Dashboard testing: toggle while running to simulate camera detection.
    public static boolean FAKE_APRIL_TAG_PRESENT = true;
    public static double VELOCITY_TOLERANCE = 30.0;
    public static double VELOCITY_STABLE_SECONDS = 0.15;
    public static double AUTO_DURATION_SECONDS = 30.0;
    public static double PARK_REMAINING_SECONDS = 8.0;

    private double targetHeadingDegrees = 0.0;
    private String autoStage = "Initializing";
    private boolean driving = false;

    private Action instant(Runnable command) {
        return packet -> {
            command.run();
            return false;
        };
    }

    // Create the trajectory only when this action is reached, so branching
    // and an interrupted path always start from the latest estimated pose.
    private Action driveTo(MecanumDrive drive, double x, double y, double headingDegrees) {
        return new Action() {
            private Action trajectory;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (trajectory == null) {
                    drive.updatePoseEstimate();
                    targetHeadingDegrees = headingDegrees;
                    driving = true;
                    trajectory = drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(
                                    new Vector2d(x, y),
                                    Math.toRadians(headingDegrees))
                            .build();
                }
                boolean running = trajectory.run(packet);
                if (!running) driving = false;
                return running;
            }
        };
    }

    // Gate may open only after a stable full-speed reading and, when required,
    // a positive simulated AprilTag result.
    private Action waitForLaunchReady(Launcher launcher, boolean requireTag) {
        return new Action() {
            private double stableSince = -1.0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double requested = launcher.getPollenLauncherTargetVelocity();
                double measured = launcher.getPollenLauncherVelocity();
                boolean fullSpeedCommanded = requested > 0 &&
                        Math.abs(requested - Launcher.LAUNCHER_VELOCITY) < 0.001;
                boolean speedValid = fullSpeedCommanded &&
                        Math.abs(measured - requested) <= VELOCITY_TOLERANCE;
                boolean tagValid = !requireTag || FAKE_APRIL_TAG_PRESENT;
                double now = System.nanoTime() / 1_000_000_000.0;

                packet.put("launch target", requested);
                packet.put("launch measured", measured);
                packet.put("launch speed valid", speedValid);
                packet.put("AprilTag valid (simulated)", tagValid);

                if (!speedValid || !tagValid) {
                    stableSince = -1.0;
                    return true;
                }
                if (stableSince < 0) stableSince = now;
                return now - stableSince < VELOCITY_STABLE_SECONDS;
            }
        };
    }

    private Action launchWhenReady(Launcher launcher, boolean requireTag, String stage) {
        return new SequentialAction(
                instant(() -> autoStage = stage + " - validating"),
                waitForLaunchReady(launcher, requireTag),
                instant(() -> autoStage = stage + " - launching"),
                launcher.LaunchFourPollenAction()
        );
    }

    private Action park(MecanumDrive drive, Launcher launcher, Intake intake) {
        return new SequentialAction(
                instant(() -> {
                    autoStage = "Parking";
                    intake.stop();
                    intake.rampsUp();
                    launcher.stopPollenLauncher();
                }),
                driveTo(drive, 50, -52, 90),
                driveTo(drive, -24, -48, 0),
                new SleepAction(1.0)
        );
    }

    private void showTelemetry(MecanumDrive drive, Launcher launcher,
                               ElapsedTime clock, boolean parking) {
        double heading = Math.toDegrees(drive.localizer.getPose().heading.toDouble());
        heading = (heading % 360 + 360) % 360;
        telemetry.addData("Stage", autoStage);
        telemetry.addData("Parking", parking);
        telemetry.addData("Time Remaining", "%.1f", Math.max(0, AUTO_DURATION_SECONDS - clock.seconds()));
        telemetry.addData("Target Heading", "%.1f", targetHeadingDegrees);
        telemetry.addData("Actual Heading", "%.1f", heading);
        telemetry.addData("Launcher Target Velocity", "%.1f", launcher.getPollenLauncherTargetVelocity());
        telemetry.addData("Launcher Actual Velocity", "%.1f", launcher.getPollenLauncherVelocity());
        telemetry.addData("AprilTag Present (FAKE)", FAKE_APRIL_TAG_PRESENT);
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        // MecanumDrive constructs PinpointLocalizer; its constructor resets
        // Pinpoint and associates the given starting field pose.
        Pose2d startPose = new Pose2d(63.5, -10.5, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Launcher launcher = new Launcher(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ElapsedTime autoClock = new ElapsedTime();
        while (opModeInInit() && !isStopRequested()) {
            drive.updatePoseEstimate();
            telemetry.addData("Stage", "Ready");
            telemetry.addData("AprilTag Present (FAKE)", FAKE_APRIL_TAG_PRESENT);
            telemetry.addData("Launcher Target Velocity", "%.1f", launcher.getPollenLauncherTargetVelocity());
            telemetry.addData("Launcher Actual Velocity", "%.1f", launcher.getPollenLauncherVelocity());
            telemetry.update();
            sleep(50);
        }
        waitForStart();
        if (isStopRequested()) return;
        autoClock.reset();

        Action normalSequence = new SequentialAction(
                instant(() -> autoStage = "First launch position"),
                new ParallelAction(
                        launcher.startPollenLauncherAction(),
                        intake.startIntakeAction(),
                        driveTo(drive, 60, -10.5, 0)
                ),
                launchWhenReady(launcher, false, "First launch"),

                instant(() -> autoStage = "Back to wall"),
                new ParallelAction(
                        launcher.idlePollenLauncherAction(),
                        driveTo(drive, 63.5, -20.5, 270)
                ),
                instant(() -> autoStage = "Garden intake"),
                driveTo(drive, 63.5, -63.5, 270),
                new SleepAction(0.5),

                instant(() -> autoStage = "Returning to launch A"),
                new ParallelAction(
                        launcher.startPollenLauncherAction(),
                        driveTo(drive, 60, -10.5, 0)
                ),
                launchWhenReady(launcher, true, "Second launch"),

                instant(() -> autoStage = "Flower pickup start"),
                new ParallelAction(
                        launcher.idlePollenLauncherAction(),
                        driveTo(drive, 24, -45, 270)
                ),
                intake.rampsDownAction(),
                instant(() -> autoStage = "Flower empty position"),
                driveTo(drive, 24, -55, 270),
                instant(() -> autoStage = "Returning to flower start"),
                driveTo(drive, 24, -45, 270),

                instant(() -> autoStage = "Returning for third launch"),
                new ParallelAction(
                        intake.rampsUpAction(),
                        launcher.startPollenLauncherAction(),
                        driveTo(drive, 60, -10.5, 0)
                ),
                launchWhenReady(launcher, true, "Third launch")
        );

        // Supervisor owns both paths. It stops invoking the old path when
        // parking is triggered, even during an AprilTag/velocity wait or drive.
        Action supervised = new Action() {
            private Action active = normalSequence;
            private boolean parking = false;
            private boolean finished = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (finished || isStopRequested()) {
                    drive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(
                            new Vector2d(0, 0), 0));
                    return false;
                }

                double remaining = AUTO_DURATION_SECONDS - autoClock.seconds();
                if (!parking && remaining <= PARK_REMAINING_SECONDS) {
                    // An abandoned trajectory no longer gets run. Explicitly
                    // stop its motor commands before creating the park path.
                    drive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(
                            new Vector2d(0, 0), 0));
                    driving = false;
                    parking = true;
                    active = park(drive, launcher, intake);
                }

                boolean running = active.run(packet);
                if (!running) {
                    if (!parking) {
                        // Normal sequence completed: park immediately.
                        parking = true;
                        active = park(drive, launcher, intake);
                        return true;
                    }
                    finished = true;
                    drive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(
                            new Vector2d(0, 0), 0));
                    launcher.stopPollenLauncher();
                    intake.stop();
                    return false;
                }

                // Update launcher PIDF and telemetry without a second
                // localization update while Road Runner is driving.
                launcher.updatePollenLauncher();
                if (!driving) drive.updatePoseEstimate();
                showTelemetry(drive, launcher, autoClock, parking);
                return true;
            }
        };

        try {
            Actions.runBlocking(supervised);
        } finally {
            drive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(
                    new Vector2d(0, 0), 0));
            launcher.stopPollenLauncher();
            intake.stop();
            intake.rampsUp();
        }
    }
}
