
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
    // CONFIGURATION
    // ========================================

    // Set TRUE after installing the gate servo.
    public static boolean USE_POLLEN_GATE = false;

    // Starting and launching position
    public static double START_X = 63.5;
    public static double START_Y = -15;
    public static double START_HEADING = 0;

    // Garden collection
    public static double GARDEN_X = 62.5;
    public static double GARDEN_START_Y = -48;
    public static double GARDEN_END_Y = -62;
    public static double GARDEN_HEADING = 270;

    // Parking waypoint
    public static double PARK_WAYPOINT_X = 66;
    public static double PARK_WAYPOINT_Y = -52;
    public static double PARK_WAYPOINT_HEADING = 0;

    // Final parking position
    public static double PARK_X = -20;
    public static double PARK_Y = -52;
    public static double PARK_HEADING = 270;

    // Launcher settings
    public static double VELOCITY_TOLERANCE = 40;
    public static double LAUNCHER_TIMEOUT = 8.0;

    // Collection pause
    public static double GARDEN_WAIT_TIME = 1.0;

    // ========================================
    // ROBOT SYSTEMS
    // ========================================

    private MecanumDrive drive;
    private Launcher launcher;
    private Intake intake;

    // ========================================
    // AUTONOMOUS
    // ========================================

    @Override
    public void runOpMode() throws InterruptedException {

        // ====================================
        // STARTING POSE
        // ====================================

        Pose2d startPose = new Pose2d(
                START_X,
                START_Y,
                Math.toRadians(START_HEADING)
        );

        Pose2d gardenStartPose = new Pose2d(
                GARDEN_X,
                GARDEN_START_Y,
                Math.toRadians(GARDEN_HEADING)
        );

        Pose2d gardenEndPose = new Pose2d(
                GARDEN_X,
                GARDEN_END_Y,
                Math.toRadians(GARDEN_HEADING)
        );

        Pose2d parkWayPointPose = new Pose2d(
                PARK_WAYPOINT_X,
                PARK_WAYPOINT_Y,
                Math.toRadians(PARK_WAYPOINT_HEADING)
        );

        Pose2d parkPose = new Pose2d(
                PARK_X,
                PARK_Y,
                Math.toRadians(PARK_HEADING)
        );

        // ====================================
        // INITIALIZE HARDWARE
        // ====================================

        drive = new MecanumDrive(hardwareMap, startPose);
        launcher = new Launcher(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        launcher.stopPollenLauncher();
        intake.stop();

        if (USE_POLLEN_GATE) {
            launcher.closePollenGate();
        }


        // ========================================
        // ROAD RUNNER TRAJECTORIES
        // ========================================

        // Travel from launch position to garden.
        // Change heading from 0 to 270 degrees.
        Action driveToGarden =
                drive.actionBuilder(startPose)
                        .strafeToLinearHeading(
                                gardenStartPose.position,
                                gardenStartPose.heading.toDouble()
                        )
                        .build();

        // ========================================
        // COLLECT GARDEN POLLEN
        // ========================================

        // Drive along garden wall.
        // Maintain 270 degree heading.
        Action collectGardenPollen =
                drive.actionBuilder(gardenStartPose)
                        .strafeTo(
                                gardenEndPose.position
                        )
                        .build();

        // ========================================
        // RETURN TO LAUNCH POSITION
        // ========================================

        // Return to original launching position.
        // Change heading back to 0 degrees.
        Action returnToLaunch =
                drive.actionBuilder(gardenEndPose)
                        .strafeToLinearHeading(
                                startPose.position,
                                startPose.heading.toDouble()
                        )
                        .build();

        // ========================================
        // PARKING ROUTE WITH WAYPOINT
        // ========================================

        // First travel to the parking waypoint
        // at (66, -52, 0 degrees).
        //
        // Then travel to the final parking position
        // at (-20, -52, 270 degrees).
        Action driveToPark =
                drive.actionBuilder(startPose)

                        .strafeToLinearHeading(
                                parkWayPointPose.position,
                                parkWayPointPose.heading.toDouble()
                        )

                        .strafeToLinearHeading(
                                parkPose.position,
                                parkPose.heading.toDouble()
                        )

                        .build();

        // ====================================
        // READY MESSAGE
        // ====================================

        telemetry.addLine("TURNING GEARS RED RIGHT");
        telemetry.addData("Gate Installed", USE_POLLEN_GATE);
        telemetry.addData(
                "Start",
                "(%.1f, %.1f, %.0f)",
                START_X,
                START_Y,
                START_HEADING
        );
        telemetry.addData(
                "Parking Waypoint",
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
        telemetry.addLine("READY");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // ====================================
        // EXECUTE AUTONOMOUS
        // ====================================

        try {

            // Intake reverse = collect pollen.
            intake.reverse();

            Action autonomousRoutine =
                    new SequentialAction(

                            // ------------------------
                            // FIRST LAUNCH
                            // ------------------------

                            launcher.startPollenLauncherAction(),

                            waitForLauncherSpeed(),

                            releaseFourPollenAction(),

                            // ------------------------
                            // LAUNCHER IDLE
                            // ------------------------

                            packet -> {
                                launcher.idlePollenLauncher();
                                return false;
                            },

                            // ------------------------
                            // DRIVE TO GARDEN
                            // ------------------------

                            driveToGarden,

                            // ------------------------
                            // COLLECT POLLEN
                            // ------------------------

                            collectGardenPollen,

                            new SleepAction(
                                    GARDEN_WAIT_TIME
                            ),

                            // ------------------------
                            // RETURN LAUNCHER TO 1180
                            // ------------------------

                            launcher.startPollenLauncherAction(),

                            // ------------------------
                            // RETURN TO LAUNCH
                            // ------------------------

                            returnToLaunch,

                            // ------------------------
                            // SECOND LAUNCH
                            // ------------------------

                            waitForLauncherSpeed(),

                            releaseFourPollenAction(),

                            // ------------------------
                            // STOP LAUNCHER AND INTAKE
                            // ------------------------

                            launcher.stopPollenLauncherAction(),

                            packet -> {
                                intake.stop();
                                return false;
                            },

                            // ------------------------
                            // PARK
                            // ------------------------

                            driveToPark
                    );

            // ====================================
            // ROAD RUNNER ACTION LOOP
            // ====================================

            while (opModeIsActive()) {

                // Maintain launcher velocity and PIDF
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

                boolean running =
                        autonomousRoutine.run(packet);

                if (!running) {
                    break;
                }

                idle();
            }

        } finally {

            // ====================================
            // SHUTDOWN
            // ====================================

            if (USE_POLLEN_GATE) {
                launcher.closePollenGate();
            }

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
    // RELEASE FOUR POLLEN
    // ========================================

    private Action releaseFourPollenAction() {

        if (USE_POLLEN_GATE) {

            // Use the existing gate method
            // from Launcher.java.
            return new SequentialAction(

                    packet -> {
                        launcher.openPollenGate();
                        return false;
                    },

                    new SleepAction(
                            Launcher.AUTO_GATE_OPEN_TIME
                    ),

                    packet -> {
                        launcher.closePollenGate();
                        return false;
                    }
            );
        }

        // Gate not installed:
        // simulate release time only.
        return new SleepAction(
                Launcher.AUTO_GATE_OPEN_TIME
        );
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

                packet.put(
                        "Target Velocity",
                        target
                );

                packet.put(
                        "Actual Velocity",
                        actual
                );

                double difference =
                        Math.abs(target - actual);

                if (difference <= VELOCITY_TOLERANCE) {
                    return false;
                }

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
