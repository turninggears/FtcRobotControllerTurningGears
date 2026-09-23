package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Chassis Drive", group = "Test")
public class ChassisDrive extends OpMode {

    MecanumDrive drive;

    @Override
    public void init() {

        Pose2d startPose = new Pose2d(
                0,
                0,
                0
        );

        drive = new MecanumDrive(
                hardwareMap,
                startPose
        );
    }

    @Override
    public void loop() {

        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(
                                forward,
                                strafe
                        ),
                        turn
                )
        );

        drive.updatePoseEstimate();

        telemetry.addData(
                "X",
                drive.localizer.getPose().position.x
        );

        telemetry.addData(
                "Y",
                drive.localizer.getPose().position.y
        );

        telemetry.addData(
                "Heading",
                Math.toDegrees(
                        drive.localizer.getPose().heading.toDouble()
                )
        );

        telemetry.update();
    }
}