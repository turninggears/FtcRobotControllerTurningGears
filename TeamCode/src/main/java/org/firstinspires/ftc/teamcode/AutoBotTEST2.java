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
@Autonomous(name = "AutoBotTEST2", group = "Autonomous")
public class AutoBotTEST2 extends LinearOpMode {

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

    public Action Pause() {
        return new Pause(0.5);
    }

    public static class Launcher {
        private Servo launchTrigger;
        private Servo artifactStopper;
        private DcMotorEx launcherMotor;
        private DcMotorEx intakeMotor;
        private DcMotor turretMotor;


        public Launcher (HardwareMap hardwareMap) {
            launchTrigger = hardwareMap.get(Servo.class,"launch trigger");
            artifactStopper = hardwareMap.get(Servo.class,"artifact stopper");
            turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
            launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher motor");
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakemotor");
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class SetTurretPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int turretTargetPosition = 940;
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setTargetPosition(turretTargetPosition);


                return false;
            }
        }

        public Action InitializeTurret() {
            return new SetTurretPosition();
        }

        public class PowerUpLauncher implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double launcherPower = 0.5;
                double intakePower = 1;
                launcherMotor.setPower(launcherPower);
                intakeMotor.setPower(intakePower);
                return false;
            }
        }

        public Action InitializeLauncher() {
            return new PowerUpLauncher();
        }

        public class Launch implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // TODO: Launcher run logic goes here
                double launchTriggerPosition = 0.9;
                double artifactStopperPosition = 0;
                launchTrigger.setPosition(launchTriggerPosition);
                artifactStopper.setPosition(artifactStopperPosition);

                return (launchTrigger.getPosition() != launchTriggerPosition && artifactStopper.getPosition() != artifactStopperPosition);
            }
        }

        public Action FireArtifact() {
            return new Launch();
        }

        public class Reset implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double launchTriggerPosition = 0.3;
                double artifactStopperPosition = 0.45;
                launchTrigger.setPosition(launchTriggerPosition);
                artifactStopper.setPosition(artifactStopperPosition);

                return (launchTrigger.getPosition() != launchTriggerPosition && artifactStopper.getPosition() != artifactStopperPosition);
            }
        }

        public Action ResetLauncher() {
            return new Reset();
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
        Launcher launcher = new Launcher(hardwareMap);
        Vector2d vector = new Vector2d(36, 15.84);
        Pause pause = new Pause(0.5);

        Action firstRow = drive.actionBuilder(startPose)
                .setTangent(Math.toRadians(0))
                .strafeTo(new Vector2d(37, 28.00)) //first row start
                //.waitSeconds(1)
                .setTangent(Math.toRadians(90))
                //.waitSeconds(1)
                .lineToY(46)  //first row intake
                //.waitSeconds(1)
                .strafeTo(new Vector2d(54.38, 15.84)) //launch spot
                .waitSeconds(1)
                .build();

        Action secondRow = drive.actionBuilder(new Pose2d(54.38, 15.84, Math.toRadians(90)))
                .strafeTo(new Vector2d(15.00, 28.00)) //second row spot
                .waitSeconds(0.1)
                .lineToY(46) //second row intake
                //.waitSeconds(1)
                .strafeTo(new Vector2d(54.38, 15.84))  //launch spot
                .waitSeconds(1)
                .build();

        Action thirdRow = drive.actionBuilder(new Pose2d(54.38, 15.84, Math.toRadians(90)))
                .strafeTo(new Vector2d(-8.00, 28.00)) //third row spot
                .waitSeconds(1)
                .lineToY(46) //third row intake
                //.waitSeconds(1)
                .strafeTo(new Vector2d(54.38, 15.84))  //launch spot
                .waitSeconds(1)
                .strafeTo(new Vector2d(64.00, 33.50))  //launch spot
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
                        launcher.InitializeLauncher(),
                        launcher.InitializeTurret(),
                        firstRow,
                        launcher.FireArtifact(),
                        Pause.pause(0.5),
                        launcher.ResetLauncher(),
                        secondRow,
                        launcher.FireArtifact(),
                        Pause.pause(0.5),
                        launcher.ResetLauncher(),
                        thirdRow,
                        launcher.FireArtifact(),
                        Pause.pause(0.5),
                        launcher.ResetLauncher()
                )
        );
    }
}