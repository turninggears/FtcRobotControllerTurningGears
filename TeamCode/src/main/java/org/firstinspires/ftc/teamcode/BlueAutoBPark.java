package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.System.PIDF;

@Config
@Autonomous(name = "BlueAutoBPark", group = "Autonomous")
public class BlueAutoBPark extends LinearOpMode {

    public Pose2d getCurrentPose(MecanumDrive drive) {
        return drive.localizer.getPose();
    }

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
            launchTrigger   = hardwareMap.get(Servo.class,"launch trigger");
            artifactStopper = hardwareMap.get(Servo.class,"artifact stopper");
            turretMotor     = hardwareMap.get(DcMotor.class, "turretMotor");
            launcherMotor   = hardwareMap.get(DcMotorEx.class, "launcher motor");
            intakeMotor     = hardwareMap.get(DcMotorEx.class, "intakemotor");
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcherMotor.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(
                            PIDF.P,
                            PIDF.I,
                            PIDF.D,
                            PIDF.F)
            );
        }

        public class SetTurretPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int turretTargetPosition = 135;
                turretMotor.setTargetPosition(turretTargetPosition);
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(.55);
                return false;
            }
        }

        public Action InitializeTurret() {
            return new SetTurretPosition();
        }

        public class PowerUpLauncher implements Action {
            double launcherVelocity;
            public PowerUpLauncher() {
                this(875);
            }

            public PowerUpLauncher(double velocity) {
                launcherVelocity = velocity;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double intakePower = 1;
                launcherMotor.setVelocity(launcherVelocity);
                intakeMotor.setPower(intakePower);
                return launcherMotor.getVelocity() < 600;
            }
        }

        public Action InitializeLauncher() {
            return new PowerUpLauncher();
        }

        public Action InitializeLauncher(double velocity) {
            return new PowerUpLauncher(velocity);
        }

        public class Launch implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double launchTriggerPosition = 0.9;
                double artifactStopperPosition = 0;
                artifactStopper.setPosition(artifactStopperPosition);
                launchTrigger.setPosition(launchTriggerPosition);

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
                double artifactStopperPosition = 0.45;//was.45
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
        Pose2d startPose = new Pose2d(-64, -39.84, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Launcher launcher = new Launcher(hardwareMap);
        Vector2d launchPosition = new Vector2d(-14, -17.84);
        Vector2d endPosition = new Vector2d(-36, -52);

        TrajectoryActionBuilder moveToLaunchPosition = drive.actionBuilder(getCurrentPose(drive))
                .strafeTo(launchPosition);

        TrajectoryActionBuilder moveToEndPosition = moveToLaunchPosition.fresh()
                .strafeTo(new Vector2d(-36, -17.84))
                .strafeTo(endPosition);

        // actions that need to happen on init

/*        while (!isStopRequested() && !opModeIsActive()) {

        }*/

        // any logic that we want to run once before the OpMode starts
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        launcher.ResetLauncher(),
                        launcher.InitializeTurret(),
                        launcher.InitializeLauncher(),
                        moveToLaunchPosition.build(),
                        Pause.pause(0.905),
                        launcher.FireArtifact(),//first artifact
                        Pause.pause(0.25),
                        launcher.ResetLauncher(),
                        Pause.pause(0.905),
                        launcher.FireArtifact(),//second artifact
                        Pause.pause(0.25),
                        launcher.ResetLauncher(),
                        Pause.pause(0.905),
                        launcher.FireArtifact(),//third artifact
                        Pause.pause(0.25),
                        launcher.ResetLauncher(),
                        Pause.pause(0.905),//should be able to remove this line eventually
                        moveToEndPosition.build()
                )
        );
        Pose2d finalPose = drive.localizer.getPose();

        telemetry.addLine("=== AUTO COMPLETE ===");
        telemetry.addData("X", finalPose.position.x);
        telemetry.addData("Y", finalPose.position.y);
        telemetry.addData("Heading (deg)", Math.toDegrees(finalPose.heading.toDouble()));
        telemetry.addData("Launcher Velocity", launcher.launcherMotor.getVelocity());
        telemetry.addData("Turret Position", launcher.turretMotor.getCurrentPosition());
        telemetry.update();

        blackboard.put("x", finalPose.position.x);
        blackboard.put("y", finalPose.position.y);
        blackboard.put("heading", finalPose.heading.toDouble());
        blackboard.put("team","blue");
        sleep(5000);
    }
}