package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.SleepAction;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.System.Launcher;
import org.firstinspires.ftc.teamcode.System.Intake;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "AutoTesting", group = "Autonomous")
public class AutoTesting extends LinearOpMode {


    public Action monitorHeading(
            MecanumDrive drive,
            double targetHeadingDegrees
    ) {
        return packet -> {

            double actualHeading = Math.toDegrees(
                    drive.localizer.getPose().heading.toDouble()
            );

            double headingError = Math.toDegrees(
                    Math.atan2(
                            Math.sin(Math.toRadians(targetHeadingDegrees - actualHeading)),
                            Math.cos(Math.toRadians(targetHeadingDegrees - actualHeading))
                    )
            );

            telemetry.addData("Target Heading", "%.1f", targetHeadingDegrees);
            telemetry.addData("Actual Heading", "%.1f", actualHeading);
            telemetry.addData("Heading Error", "%.1f", headingError);
            telemetry.update();

            packet.put("Target Heading", targetHeadingDegrees);
            packet.put("Actual Heading", actualHeading);
            packet.put("Heading Error", headingError);

            return false;
        };
    }



    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(63.5, -10.5, Math.toRadians(0));
//        Pose2d endPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Launcher launcher = new Launcher(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        TrajectoryActionBuilder launchPosition = drive.actionBuilder(startPose)
                .setTangent(Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(60, -10.5),
                        Math.toRadians(0))
                .waitSeconds(2)
                .endTrajectory();

        TrajectoryActionBuilder backToWall = launchPosition.fresh()
                .strafeToLinearHeading(
                        new Vector2d(65, -10.5),
                Math.toRadians(270))
                .endTrajectory();

        TrajectoryActionBuilder parkPosition = backToWall.fresh()
                .setTangent(Math.toRadians(270))
                .strafeTo(new Vector2d(65, -60))
                .endTrajectory();


        List<Action> actions = new ArrayList<>();

        //Start Launcher
        actions.add(launcher.startPollenLauncherAction());

// Move to launch position
        actions.add(launchPosition.build());

// Check heading after the turn
        actions.add(monitorHeading(drive, 270));

// Move back to wall
        actions.add(backToWall.build());

// Check heading after moving back to wall
        actions.add(monitorHeading(drive, 270));

// Move to parking position
        actions.add(parkPosition.build());

// Check final heading
        actions.add(monitorHeading(drive, 270));


        // actions that need to happen on init

/*        while (!isStopRequested() && !opModeIsActive()) {
// any logic while the robot is running but OpMode ius not yet active

        }*/

        // any logic that we want to run once before the OpMode starts
        waitForStart();

        if (isStopRequested()) return;



        Actions.runBlocking(
                new SequentialAction(actions)
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