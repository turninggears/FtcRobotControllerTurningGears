
package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.SleepAction;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.System.Launcher;

@Autonomous(name = "AutoTesting", group = "Autonomous")
public class AutoTesting extends LinearOpMode {


    private void updateRobotTelemetry(
            MecanumDrive drive,
            Launcher launcher,
            double targetHeadingDegrees
    ) {
        double actualHeading = Math.toDegrees(
                drive.localizer.getPose().heading.toDouble()
        );

        actualHeading = (actualHeading % 360 + 360) % 360;

        telemetry.addData("Target Heading",
                "%.1f", targetHeadingDegrees);
        telemetry.addData("Actual Heading",
                "%.1f", actualHeading);
        telemetry.addData("Launcher Target Velocity",
                "%.1f", launcher.getPollenLauncherTargetVelocity());
        telemetry.addData("Launcher Actual Velocity",
                "%.1f", launcher.getPollenLauncherVelocity());

        telemetry.update();
    }

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(
                63.5, -10.5, Math.toRadians(0)
        );

        MecanumDrive drive = new MecanumDrive(
                hardwareMap, startPose
        );

        Launcher launcher = new Launcher(
                hardwareMap, telemetry
        );

        // Send telemetry to both Driver Station and Dashboard
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        final double[] targetHeadingDegrees = {0};
        final boolean[] driveFinished = {false};

        // TRAJECTORY 1 - LAUNCH POSITION
        TrajectoryActionBuilder launchPosition =
                drive.actionBuilder(startPose)
                        .setTangent(Math.toRadians(0))
                        .strafeToLinearHeading(
                                new Vector2d(60, -10.5),
                                Math.toRadians(0)
                        )
                        .waitSeconds(2);

        // TRAJECTORY 2 - BACK TO WALL
        TrajectoryActionBuilder backToWall =
                launchPosition.fresh()
                        .turnTo(Math.toRadians(255));

        // TRAJECTORY 3 - PARK
        TrajectoryActionBuilder parkPosition =
                backToWall.fresh()
                        .setTangent(Math.toRadians(270))
                        .strafeTo(
                                new Vector2d(65, -60)
                        );

        // INIT TELEMETRY - BEFORE START IS PRESSED
        while (opModeInInit() && !isStopRequested()) {
            drive.updatePoseEstimate();

            updateRobotTelemetry(
                    drive, launcher, targetHeadingDegrees[0]
            );

            sleep(50);
        }

        waitForStart();

        if (isStopRequested()) return;

        // Preserve the original 3-second delay
        long delayStart = System.currentTimeMillis();

        while (opModeIsActive()
                && System.currentTimeMillis() - delayStart < 3000) {

            drive.updatePoseEstimate();

            updateRobotTelemetry(
                    drive, launcher, targetHeadingDegrees[0]
            );

            sleep(50);
        }

        if (isStopRequested()) return;

        // DRIVING AND LAUNCHER ACTIONS
        Action driveSequence = new SequentialAction(

                launcher.startPollenLauncherAction(),

                launchPosition.build(),

                packet -> {
                    targetHeadingDegrees[0] = 270;
                    return false;
                },

                backToWall.build(),

                new SleepAction(2),

                parkPosition.build(),

                packet -> {
                    driveFinished[0] = true;
                    return false;
                }
        );

        // CONTINUOUS TELEMETRY MONITOR
        Action telemetryMonitor = packet -> {

            if (driveFinished[0] || isStopRequested()) {
                return false;
            }

            // Apply live launcher velocity and PIDF changes
            launcher.updatePollenLauncher();

            updateRobotTelemetry(
                    drive, launcher, targetHeadingDegrees[0]
            );

            return true;
        };

        // RUN TRAJECTORIES AND TELEMETRY TOGETHER
        Actions.runBlocking(
                new ParallelAction(
                        driveSequence,
                        telemetryMonitor
                )
        );

        // Stop launcher after autonomous movement
        launcher.stopPollenLauncher();

        // Hold final telemetry on screen
        updateRobotTelemetry(
                drive, launcher, targetHeadingDegrees[0]
        );

        sleep(5000);
    }
}
