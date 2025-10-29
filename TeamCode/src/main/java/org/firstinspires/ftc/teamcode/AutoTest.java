package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "AutoTest", group = "Autonomous")
public class AutoTest extends LinearOpMode {

    public class Launcher {
        private final DcMotorEx launcherMotor;

        public Launcher(HardwareMap hardwareMap) {
            launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher motor");
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launcherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        /**
         * Launch action RUNS for a fixed time and then ENDS properly.
         */
        public class Launch implements Action {
            private boolean initialized = false;
            private final ElapsedTime timer = new ElapsedTime();
            private static final double RUN_TIME_SEC = 1.5; // how long launcher runs
            private static final double POWER = 1.0;        // launcher power

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    launcherMotor.setPower(POWER);
                    timer.reset();
                    initialized = true;
                }

                packet.put("Launcher Power", launcherMotor.getPower());
                packet.put("Launcher Time (s)", timer.seconds());

                if (timer.seconds() < RUN_TIME_SEC) {
                    return true; // keep running launcher
                } else {
                    launcherMotor.setPower(0.0);
                    return false; // END action so sequence can continue
                }
            }
        }

        public Action launch() {
            return new Launch();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(50, 0, Math.toRadians(0));
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

        Action end = trajectory.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        Action position = trajectory.build();

        while (!isStopRequested() && !opModeIsActive()) {
            // idle loop while waiting for start
        }

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        position,          // path 1
                        launcher.launch(), // fixed launcher action
                        end                // final path
                )
        );
    }
}
