package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.System.Launcher;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "BioBuzz Auto", group = "Autonomous")
public class BioBuzzAuto extends LinearOpMode {

    Launcher launcher = new Launcher(hardwareMap, telemetry);

    public static class Pause implements Action {

        public static Pause pause(double seconds) {
            return new Pause(seconds);
        }
        double seconds;
        long startTime = -1;
        public Pause(double seconds) {
            this.seconds = seconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (startTime < 0) {
                startTime = System.currentTimeMillis();
            }
            return System.currentTimeMillis() < startTime + (seconds * 1000);
        }


    }

    public Action PauseAction() {
        return new Pause(1.0);
    }


    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(63.5, -10.5, Math.toRadians(0));
//        Pose2d endPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

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

//        if (isStopRequested()) return;



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