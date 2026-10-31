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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.System.Launcher;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "BioBuzz Auto V3", group = "Autonomous")
public class BioBuzzAutoV3 extends LinearOpMode {



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

        Pose2d startPose = new Pose2d(-63.5, -10.5, Math.toRadians(180));
//        Pose2d endPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Launcher launcher = new Launcher(hardwareMap, telemetry);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        TrajectoryActionBuilder launchPositionB = drive.actionBuilder(startPose)
                .setTangent(Math.toRadians(0))
                .strafeTo(new Vector2d(-60, -10.5))
                .waitSeconds(1)
                .endTrajectory();

        TrajectoryActionBuilder flowerBLineUp = launchPositionB.fresh()
                .strafeTo(new Vector2d(-55, -24))
                .endTrajectory();

        TrajectoryActionBuilder flowerBPickup = flowerBLineUp.fresh()
                .setTangent(Math.toRadians(180))
                .strafeTo(new Vector2d(-58, -24))
                .endTrajectory();

        TrajectoryActionBuilder secondLaunchB = flowerBPickup.fresh()
                .strafeTo(new Vector2d(-60, -13.5))
                .waitSeconds(1)
                .endTrajectory();

        TrajectoryActionBuilder parkPositionB = secondLaunchB.fresh()
                .strafeToLinearHeading(
                        new Vector2d(-48,-56),
                    Math.toRadians(90)
                                )
        .endTrajectory();









        List<Action> actions = new ArrayList<>();
        actions.add(launchPositionB.build());
        actions.add(flowerBLineUp.build());
        actions.add(flowerBPickup.build());
        actions.add(secondLaunchB.build());
        actions.add(parkPositionB.build());

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