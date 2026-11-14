package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Pinpoint Push Test", group = "Testing")
public class PinpointPushTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(0, 0, 0);

        MecanumDriveBiobuzz drive =
                new MecanumDriveBiobuzz(hardwareMap, startPose);

        telemetry.addLine("Ready for Pinpoint Test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            drive.updatePoseEstimate();

            Pose2d pose = drive.localizer.getPose();

            telemetry.addData("X Position", "%.2f",
                    pose.position.x);

            telemetry.addData("Y Position", "%.2f",
                    pose.position.y);

            telemetry.addData("Heading", "%.1f",
                    Math.toDegrees(pose.heading.toDouble()));

            telemetry.update();
        }
    }
}