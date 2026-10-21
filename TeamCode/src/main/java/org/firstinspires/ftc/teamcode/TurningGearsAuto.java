
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.System.Launcher;
import org.firstinspires.ftc.teamcode.System.Intake;

@Config
@Autonomous(
        name = "Turning Gears Red Right",
        group = "Competition"
)
public class TurningGearsAuto extends LinearOpMode {

    // ========================================
    // STARTING / LAUNCH POSITION
    // ========================================

    public static double START_X = 66;
    public static double START_Y = -15;
    public static double START_HEADING = 0;

    // ========================================
    // GARDEN COLLECTION
    // ========================================

    public static double GARDEN_X = 62.5;
    public static double GARDEN_START_Y = -48;
    public static double GARDEN_END_Y = -62;
    public static double GARDEN_HEADING = 270;

    // ========================================
    // PARKING WAYPOINT
    // ========================================

    public static double PARK_WAYPOINT_X = 66;
    public static double PARK_WAYPOINT_Y = -52;
    public static double PARK_WAYPOINT_HEADING = 0;

    // ========================================
    // FINAL PARKING POSITION
    // ========================================

    public static double PARK_X = -20;
    public static double PARK_Y = -52;
    public static double PARK_HEADING = 270;

    // ========================================
    // LAUNCHER / COLLECTION SETTINGS
    // ========================================

    public static double VELOCITY_TOLERANCE = 40;
    public static double LAUNCHER_TIMEOUT = 4.0;
    public static double GARDEN_WAIT_TIME = 1.0;

    // ========================================
    // ROBOT HARDWARE
    // ========================================

    private MecanumDrive drive;
    private Launcher launcher;
    private Intake intake;

    // ========================================
    // AUTONOMOUS PROGRAM
    // ========================================

    @Override
    public void runOpMode() throws InterruptedException {

        // Starting position
        Pose2d startPose = new Pose2d(
                START_X,
                START_Y,
                Math.toRadians(START_HEADING)
        );

        // Garden entrance
        Pose2d gardenStartPose = new Pose2d(
                GARDEN_X,
                GARDEN_START_Y,
                Math.toRadians(GARDEN_HEADING)
        );

        // Garden collection endpoint
        Pose2d gardenEndPose = new Pose2d(
                GARDEN_X,
                GARDEN_END_Y,
                Math.toRadians(GARDEN_HEADING)
        );

        // Parking waypoint
        Pose2d parkWaypointPose = new Pose2d(
                PARK_WAYPOINT_X,
                PARK_WAYPOINT_Y,
                Math.toRadians(PARK_WAYPOINT_HEADING)
        );

        // Final parking position
        Pose2d parkPose = new Pose2d(
                PARK_X,
                PARK_Y,
                Math.toRadians(PARK_HEADING)
        );

        // ========================================
        // INITIALIZE ROBOT SYSTEMS
        // ========================================

        drive = new MecanumDrive(hardwareMap, startPose);
        launcher = new Launcher(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        launcher.stopPollenLauncher();
        launcher.closePollenGate();
        intake.stop();

        // ========================================
        // ROAD RUNNER TRAJECTORIES
        // ========================================

        // Travel from launch to garden entrance.
        // Rotate from 0 to 270 degrees.
        Action driveToGarden =
                drive.actionBuilder(startPose)
                        .strafeToLinearHeading(
                                new Vector2d(
                                        GARDEN_X,
                                        GARDEN_START_Y
                                ),
                                Math.toRadians(GARDEN_HEADING)
                        )
                        .build();

        // Collect pollen while moving along wall.
        // Maintain 270 degree heading.
        Action collectGardenPollen =
                drive.actionBuilder(gardenStartPose)
                        .strafeTo(
                                new Vector2d(
                                        GARDEN_X,
                                        GARDEN_END_Y
                                )
                        )
                        .build();

        // Return to original launch position.
        // Launcher accelerates during this movement.
        Action returnToLaunch =
                drive.actionBuilder(gardenEndPose)
                        .strafeToLinearHeading(
                                new Vector2d(
                                        START_X,
                                        START_Y
                                ),
                                Math.toRadians(START_HEADING)
                        )
                        .build();

        // ========================================
        // PARKING ROUTE WITH WAYPOINT
        // ========================================

        // First move to (66, -52) at 0 degrees.
        // Then move to (-20, -52) at 270 degrees.
        Action driveToPark =
                drive.actionBuilder(startPose)

                        .strafeToLinearHeading(
                                new Vector2d(
                                        PARK_WAYPOINT_X,
                                        PARK_WAYPOINT_Y
                                ),
                                Math.toRadians(
                                        PARK_WAYPOINT_HEADING
                                )
                        )

                        .strafeToLinearHeading(
                                new Vector2d(
                                        PARK_X,
                                        PARK_Y
                                ),
                                Math.toRadians(PARK_HEADING)
                        )

                        .build();

        // ========================================
        // DRIVER STATION READY MESSAGE
        // ========================================

        telemetry.addLine("TURNING GEARS AUTONOMOUS");
        telemetry.addLine("Alliance: RED");
        telemetry.addLine("Starting Position: RIGHT");

        telemetry.addData(
                "Start",
                "(%.1f, %.1f, %.0f)",
                START_X,
                START_Y,
                START_HEADING
        );

        telemetry.addData(
                "Garden Start",
                "(%.1f, %.1f)",
                GARDEN_X,
                GARDEN_START_Y
        );

        telemetry.addData(
                "Garden End",
                "(%.1f, %.1f)",
                GARDEN_X,
                GARDEN_END_Y
        );

        telemetry.addData(
                "Park Waypoint",
                "(%.1f, %.1f, %.0f)",
                PARK_WAYPOINT_X,
                PARK_WAYPOINT_Y,
                PARK_WAYPOINT_HEADING
        );

        telemetry.addData(
                "Final Park",
                "(%.1f, %.1f, %.0f)",
                PARK_X,
                PARK_Y,
                PARK_HEADING
        );

        telemetry.addLine("READY TO START");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        try {

            // ====================================
            // START INTAKE
            // ====================================

            // Reverse direction is collection.
            // Keep running through both launches.
            intake.reverse();

            // ====================================
            // BUILD AUTONOMOUS SEQUENCE
            // ====================================

            Action autonomousRoutine = new SequentialAction(

                    // ====================================
                    // FIRST LAUNCH
                    // ====================================

                    // Start launcher at full velocity.
                    launcher.startPollenLauncherAction(),

                    // Wait for launcher to reach speed.
                    waitForLauncherSpeed(),

                    // Open gate, release four pollen,
                    // and close gate automatically.
                    launcher.launchFourPollenAction(),

                    // ====================================
                    // REDUCE LAUNCHER TO IDLE
                    // ====================================

                    packet -> {
                        launcher.idlePollenLauncher();
                        return false;
                    },

                    // ====================================
                    // DRIVE TO GARDEN
                    // ====================================

                    driveToGarden,

                    // ====================================
                    // COLLECT FOUR POLLEN
                    // ====================================

                    // Drive from Y -48 to Y -62.
                    collectGardenPollen,

                    // Allow extra collection time.
                    new SleepAction(GARDEN_WAIT_TIME),

                    // ====================================
                    // RESTORE LAUNCHER SPEED
                    // ====================================

                    // Start accelerating before return.
                    launcher.startPollenLauncherAction(),

                    // ====================================
                    // RETURN TO LAUNCH
                    // ====================================

                    returnToLaunch,

                    // ====================================
                    // SECOND LAUNCH
                    // ====================================

                    // Wait until launcher reaches speed.
                    waitForLauncherSpeed(),

                    // Release collected pollen.
                    launcher.launchFourPollenAction(),

                    // ====================================
                    // STOP LAUNCHER AND INTAKE
                    // ====================================

                    launcher.stopPollenLauncherAction(),

                    packet -> {
                        intake.stop();
                        return false;
                    },

                    // ====================================
                    // PARK THROUGH WAYPOINT
                    // ====================================

                    driveToPark
            );

            // ========================================
            // EXECUTE AUTONOMOUS
            // ========================================

            // Run Road Runner actions while
            // updating launcher velocity and PIDF.
            while (opModeIsActive()) {

                launcher.updatePollenLauncher();

                TelemetryPacket packet =
                        new TelemetryPacket();

                packet.put(
                        "Launcher Target",
                        launcher.getPollenLauncherTargetVelocity()
                );

                packet.put(
                        "Launcher Actual",
                        launcher.getPollenLauncherVelocity()
                );

                boolean stillRunning =
                        autonomousRoutine.run(packet);

                if (!stillRunning) {
                    break;
                }

                idle();
            }

        } finally {

            // ========================================
            // FINAL SHUTDOWN
            // ========================================

            launcher.closePollenGate();
            launcher.stopPollenLauncher();
            intake.stop();

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(0, 0),
                            0
                    )
            );

            telemetry.addLine("AUTONOMOUS ENDED");
            telemetry.update();
        }
    }

    // ========================================
    // WAIT FOR LAUNCHER VELOCITY
    // ========================================

    private Action waitForLauncherSpeed() {

        return new Action() {

            private long startTime = -1;

            @Override
            public boolean run(TelemetryPacket packet) {

                if (startTime == -1) {
                    startTime = System.nanoTime();
                }

                double elapsed =
                        (System.nanoTime() - startTime) / 1e9;

                double target =
                        launcher.getPollenLauncherTargetVelocity();

                double actual =
                        launcher.getPollenLauncherVelocity();

                packet.put("Target Velocity", target);
                packet.put("Actual Velocity", actual);

                double difference =
                        Math.abs(target - actual);

                // Continue when target speed is reached.
                if (difference <= VELOCITY_TOLERANCE) {
                    return false;
                }

                // Abort if launcher cannot reach speed.
                if (elapsed >= LAUNCHER_TIMEOUT) {
                    throw new IllegalStateException(
                            "Launcher failed to reach target velocity"
                    );
                }

                return true;
            }
        };
    }
}
