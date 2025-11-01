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
        /*double ROBOT_CENTER_X = 207.5;

        double ROBOT_CENTER_Y = 207.5;

        // Start position on the grid in mm

        double startPosX = 1414.3;

        double startPosY = 0;

        double ROBOT_CENTER_X_IN = ROBOT_CENTER_X / 25.4;
        double ROBOT_CENTER_Y_IN = ROBOT_CENTER_Y / 25.4;
        double start_pos_x_in = startPosX / 25.4;
        double start_pos_y_in = startPosY / 25.4;*/
        Pose2d startPose = new Pose2d(64, 15.84, Math.toRadians(90));
//        Pose2d endPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Vector2d vector = new Vector2d(36, 15.84);
//        Launcher launcher = new Launcher(hardwareMap);

        Action firstRow = drive.actionBuilder(startPose)
                .setTangent(Math.toRadians(0))
                .strafeTo(vector)
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(50)
                .strafeTo(new Vector2d(54.38, 15.84))
                .build();

//        Action position = traj.build();

        // actions that need to happen on init

//        while (!isStopRequested() && !opModeIsActive()) {
//            // any logic while the robot is running but OpMode ius not yet active
//        }

        // any logic that we want to run once before the OpMode starts
        waitForStart();

//        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        firstRow
                )
        );
    }
}