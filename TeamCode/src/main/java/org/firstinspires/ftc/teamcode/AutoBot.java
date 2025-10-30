package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
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
            return System.currentTimeMillis() > startTime + (seconds * 1000);
        }


    }

    public class Launcher {
        private DcMotorEx launcherMotor;

        public Launcher (HardwareMap hardwareMap) {
            launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher motor");
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        public class Launch implements Action {
            double launchSpeed;
            public Launch(double speed) {
                this.launchSpeed = speed;
            }

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // TODO: Launcher run logic goes here

                if (!initialized) {
                    // TODO: launcher initialization logic
                    initialized = true;
                    launcherMotor.setPower(launchSpeed);
                }



                packet.put("Launcher power: ", launchSpeed);

                return true;
            }
        }

        public Action launch(double speed) {
            return new Launch(speed);
        }

    }
    @Override
    public void runOpMode() {
        /* measurements done in millimeters but RoadRunner uses inches;
           easiest to measure in mm and then convert to inches (mm/25.4)
         */
        double ROBOT_CENTER_X = 207.5;

        double ROBOT_CENTER_Y = 207.5;

        // Start position on the grid in mm

        double startPosX = 1927;

        double startPosY = 0;

        double ROBOT_CENTER_X_IN = ROBOT_CENTER_X / 25.4;
        double ROBOT_CENTER_Y_IN = ROBOT_CENTER_Y / 25.4;
        double startPosXin = (startPosX / 25.4) + ROBOT_CENTER_X_IN;
        double startPoxYin = (startPosY / 25.4) + ROBOT_CENTER_Y_IN;
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d endPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Launcher launcher = new Launcher(hardwareMap);

        TrajectoryActionBuilder trajectory = drive.actionBuilder(startPose)
//                .setTangent(0)
//                .lineToX(33);
//                .splineToLinearHeading(endPose, 0);
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(6, Math.toRadians(0))
                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
//                .lineToY(48)
                .setTangent(Math.toRadians(180))
                .lineToXLinearHeading(-6, Math.toRadians(0));
//                .strafeTo(new Vector2d(44.5, 30))
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
//                .waitSeconds(3);


        Action end = trajectory.endTrajectory().fresh()
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

//        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        position
//                        Pause.pause(2.0),
//                        launcher.launch(0.5)
                )
        );
    }
}