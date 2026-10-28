package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.System.Launcher;
import com.acmerobotics.roadrunner.ParallelAction;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "BioBuzz Auto", group = "Autonomous")
public class BioBuzzAuto extends LinearOpMode {



    public Action monitorHeadingContinuous(
            MecanumDrive drive,
            boolean[] driveFinished
    ) {
        return packet -> {

            // Stop monitoring when trajectories finish
            if (driveFinished[0]) {
                return false;
            }

            // Read the latest Road Runner pose
            Pose2d actualPose = drive.localizer.getPose();

            double actualHeading = Math.toDegrees(
                    actualPose.heading.toDouble()
            );

            // Normalize heading to 0-360 degrees
            actualHeading = (actualHeading % 360 + 360) % 360;

            // Display on Driver Station and Dashboard
            telemetry.addData(
                    "Actual Heading (deg)",
                    "%.1f",
                    actualHeading
            );

            telemetry.update();

            // Add to Road Runner telemetry packet
            packet.put("Actual Heading (deg)", actualHeading);

            return true;
        };
    }



    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(63.5, -10.5, Math.toRadians(0));
//        Pose2d endPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Launcher launcher = new Launcher(hardwareMap, telemetry);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        TrajectoryActionBuilder launchPosition = drive.actionBuilder(startPose)
                .setTangent(Math.toRadians(0))
                .strafeTo(new Vector2d(60, -10.5))
                .waitSeconds(2)
                .turnTo(Math.toRadians(270))
                .endTrajectory();

        TrajectoryActionBuilder backToWall = launchPosition.fresh()
                .strafeTo(new Vector2d(65, -10.5))
                .endTrajectory();

        TrajectoryActionBuilder parkPosition = backToWall.fresh()
                .setTangent(Math.toRadians(270))
                .strafeTo(new Vector2d(65, -60))
                .endTrajectory();

        List<Action> actions = new ArrayList<>();
        actions.add(launchPosition.build());
        actions.add(backToWall.build());
        actions.add(parkPosition.build());

        // actions that need to happen on init

/*        while (!isStopRequested() && !opModeIsActive()) {
// any logic while the robot is running but OpMode ius not yet active

        }*/

        // any logic that we want to run once before the OpMode starts
        waitForStart();

        if (isStopRequested()) return;




        final boolean[] driveFinished = {false};

// Run all three trajectories in sequence
        Action driveSequence = new SequentialAction(
                new SequentialAction(actions),

                packet -> {
                    driveFinished[0] = true;
                    return false;
                }
        );

// Continuously monitor heading
        Action headingMonitor = monitorHeadingContinuous(
                drive,
                driveFinished
        );

// Run driving and monitoring together
        Actions.runBlocking(
                new ParallelAction(
                        driveSequence,
                        headingMonitor
                )
        );

        Pose2d finalPose = drive.localizer.getPose();

        telemetry.addLine("=== AUTO COMPLETE ===");
        telemetry.addData("X", finalPose.position.x);
        telemetry.addData("Y", finalPose.position.y);
        telemetry.addData("Heading (deg)", Math.toDegrees(finalPose.heading.toDouble()));

        telemetry.update();

        blackboard.put("x", finalPose.position.x);
        blackboard.put("y", finalPose.position.y);
        blackboard.put("heading", finalPose.heading.toDouble());
        blackboard.put("team","blue");
        sleep(5000);

    }
}