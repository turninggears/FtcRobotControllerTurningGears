
package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.System.Intake;
import org.firstinspires.ftc.teamcode.System.Launcher;
import org.firstinspires.ftc.teamcode.System.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@SuppressLint("DefaultLocale")
@TeleOp(name = "BuildTeleOp", group = "Test")
@Config
public class MotorTest extends OpMode {

    // ========================================
    // DRIVING VARIABLES
    // ========================================

    MecanumDriveEncoders drive;

    boolean fieldCentric = true;
    boolean previousTriangle = false;
    boolean previousSquare = false;
    double headingOffset = 0.0;

    int firstLoop = 1;
    // ========================================
    // VARIABLES
    // ========================================

    Launcher launcher;
    Intake intake;
    Vision vision;

    // Intake motor state
// 0 = stopped, 1 = forward, -1 = reverse
    int intakeDirection = 0;


    // ========================================
    // INITIALIZATION
    // ========================================

    @Override
    public void init() {

        // -------- DRIVETRAIN SETUP --------

        Pose2d startPose = new Pose2d(
                0,
                0,
                0
        );

        drive = new MecanumDriveEncoders(
                hardwareMap,
                startPose
        );

        // -------- DASHBOARD TELEMETRY --------

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        telemetry.update();
    }


    // ========================================
    // MAIN LOOP
    // ========================================


    @Override
    public void loop() {

        // ===============================-====
        // GAMEPAD 1 - DRIVING
        // ====================================

        if (gamepad1.triangle) {
            drive.leftFront.setPower(1);
        }

        if (gamepad1.circle) {
            drive.rightFront.setPower(1);
        }

        if (gamepad1.square) {
            drive.leftBack.setPower(1);
        }

        if (gamepad1.cross) {
            drive.rightBack.setPower(1);
        }

        double leftFrontVelocity = drive.leftFront.getVelocity();
        double leftBackVelocity = drive.leftBack.getVelocity();
        double rightFrontVelocity = drive.rightFront.getVelocity();
        double rightBackVelocity = drive.rightBack.getVelocity();

        telemetry.addLine("GAMEPAD 1:");
        telemetry.addLine("Triangle: FL Drive");
        telemetry.addLine("Circle: FR Drive");
        telemetry.addLine("Square: BL Drive");
        telemetry.addLine("Cross: BR Drive");

        telemetry.addData("FL Drive velocity: ", leftFrontVelocity);
        telemetry.addData("BL Drive velocity: ", leftBackVelocity);
        telemetry.addData("FR Drive Velocity: ", rightFrontVelocity);
        telemetry.addData("BR Drive velocity: ", rightBackVelocity);




        telemetry.update();
    }

}
