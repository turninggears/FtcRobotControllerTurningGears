package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@Autonomous(name = "AutoBot", group = "Autonomous")
public class AutoBot extends LinearOpMode {

    public class Launcher {
        private DcMotorEx launcherMotor;

        public Launcher (HardwareMap hardwareMap) {
            launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher motor");
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        public class Launch implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // TODO: Launcher run logic goes here

                if (!initialized) {
                    // TODO: launcher initialization logic
                    initialized = true;
                }

                double launcherPower = launcherMotor.getPower();

                packet.put("Launcher power: ", launcherPower);

                return true;
            }
        }

        public Action launch() {
            return new Launch();
        }

    }
    @Override
    public void runOpMode() {
        // change this to our actual starting position
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Launcher launcher = new Launcher(hardwareMap);

        TrajectoryActionBuilder trajectory = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);

        Action trajectoryActionCloseOut = trajectory.endTrajectory().fresh()
                // strafe to Autonomous end position
                .strafeTo(new Vector2d(48, 12))
                .build();

        Action position = trajectory.build();

        // actions that need to happen on init; for instance, a claw tightening.
        // Actions.runBlocking(claw.closeClaw());



        while (!isStopRequested() && !opModeIsActive()) {
            // any logic while the robot is running but OpMode ius not yet active
        }

        // any logic that we want to run once before the OpMode starts
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        position,
                        launcher.launch(),
                        trajectoryActionCloseOut
                )
        );
    }
}