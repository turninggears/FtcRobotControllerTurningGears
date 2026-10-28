package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name = "Chassis Drive", group = "Test")
public class ChassisDrive extends OpMode {

    MecanumDrive drive;

    boolean fieldCentric = true;
    boolean previousTriangle = false;
    boolean previousSquare = false;
    double headingOffset = 0.0;

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

        drive.updatePoseEstimate();

        double currentHeading =
                drive.localizer.getPose().heading.toDouble();

        //triangle: Toggle Field-centric/Robot Centric

        if (gamepad1.triangle && !previousTriangle) {
            fieldCentric = !fieldCentric;
        }
        previousTriangle = gamepad1.triangle;

        //Square: Reset current robot direction as field forward
        if (gamepad1.square && !previousSquare) {
            headingOffset = currentHeading;
        }

        previousSquare = gamepad1.square;

        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        //R1:reduce drive speed to 50% while held
        double speedMultiplier = gamepad1.right_bumper ? .5 : 1.0;

        //create drive vector from joystick input
        Vector2d driveVector = new Vector2d(forward,strafe);

        //field centric conversion
        if (fieldCentric){

            double driveHeading = currentHeading - headingOffset;

            driveVector = Rotation2d
                    .fromDouble(-driveHeading)
                    .times(driveVector);
        }


        drive.setDrivePowers(
                new PoseVelocity2d(
                        driveVector.times(speedMultiplier), 
                        turn * speedMultiplier
                )
        );

        telemetry.addData(
                "Drive Mode",
                fieldCentric ? "Field Centric" : "Robot Centric"
        );


        telemetry.addLine("");
        telemetry.addLine("Triangle = Field/Robot Centric");
        telemetry.addLine("Square = Reset Field Heading");
        telemetry.addLine("Hold R1 = 50% Speed");
        telemetry.update();
    }
}