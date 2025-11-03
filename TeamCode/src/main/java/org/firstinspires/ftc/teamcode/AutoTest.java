package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoTest", group = "Autonomous")
public class AutoTest extends LinearOpMode {

    // Road Runner drive
    private MecanumDrive drive;

    // Motors
    private DcMotorEx launcherMotor;
    private DcMotorEx turretMotor;
    private DcMotorEx intakeMotor;

    // Servos
    private Servo launchTrigger;
    private Servo artifactStopper;

    // constants
    private static final Pose2d START_POSE =
            new Pose2d(64.0, 15.84, Math.toRadians(90.0));

    private static final int TURRET_TARGET_TICKS = -270;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. hardware map
        launcherMotor   = hardwareMap.get(DcMotorEx.class, "launcher motor");
        turretMotor     = hardwareMap.get(DcMotorEx.class, "turretMotor");
        intakeMotor     = hardwareMap.get(DcMotorEx.class, "intakemotor");

        launchTrigger   = hardwareMap.get(Servo.class, "launch trigger");
        artifactStopper = hardwareMap.get(Servo.class, "artifact stopper");

        // 2. drive + pose
        // change this line if your drive has a different constructor
        drive = new MecanumDrive(hardwareMap, START_POSE);
        // or:
        // drive = new MecanumDrive(hardwareMap);
        // drive.setPose(START_POSE);

        // 3. motor setup
        //launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0);

        // 4. init servo positions
        launchTrigger.setPosition(0.3);
        artifactStopper.setPosition(0.45);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Pose", START_POSE);
        telemetry.addData("Servos", "launchTrigger=0.3, artifactStopper=0.45");
        telemetry.update();

        // 5. wait for start
        waitForStart();
        if (isStopRequested()) return;

        // 6. on play
        // launcher ON at 0.65
        launcherMotor.setPower(0.65);

        // turret to -270, power 1, hold
        turretMotor.setTargetPosition(TURRET_TARGET_TICKS);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1.0);

        // 7. define the launch sequence (3 cycles)
        Action launchSeq = buildTripleLaunchAction();

        // 8. build all motion actions in your exact order

        // start -> (36, 15.84, 90) strafe, tangent 0
        Action moveTo36_1584 = drive.actionBuilder(START_POSE)
                .strafeTo(new Vector2d(36, 15.84))
                .build();

        // (36, 15.84, 90) -> (36, 50, 90) tangent 90
        Action moveTo36_50 = drive.actionBuilder(
                        new Pose2d(36, 15.84, Math.toRadians(90.0)))
                .splineToConstantHeading(new Vector2d(36, 50), Math.toRadians(90.0))
                .build();

        // (36, 50, 90) -> back to start in a spline
        Action backToStartFrom36_50 = drive.actionBuilder(
                        new Pose2d(36, 50, Math.toRadians(90.0)))
                .splineToConstantHeading(new Vector2d(64, 15.84), Math.toRadians(-90.0))
                .build();

        // start -> (12, 15.84, 90) strafe
        Action moveTo12_1584 = drive.actionBuilder(START_POSE)
                .strafeTo(new Vector2d(12, 15.84))
                .build();

        // (12, 15.84, 90) -> (12, 50, 90) tangent 0 (forward with same heading)
        Action moveTo12_50 = drive.actionBuilder(
                        new Pose2d(12, 15.84, Math.toRadians(90.0)))
                .splineToConstantHeading(new Vector2d(12, 50), Math.toRadians(90.0))
                .build();

        // (12, 50, 90) -> back to start
        Action backToStartFrom12_50 = drive.actionBuilder(
                        new Pose2d(12, 50, Math.toRadians(90.0)))
                .splineToConstantHeading(new Vector2d(64, 15.84), Math.toRadians(-90.0))
                .build();

        // start -> (-12, 15.84, 90) strafe
        Action moveToNeg12_1584 = drive.actionBuilder(START_POSE)
                .strafeTo(new Vector2d(-12, 15.84))
                .build();

        // (-12, 15.84, 90) -> (-12, 50, 90)
        Action moveToNeg12_50 = drive.actionBuilder(
                        new Pose2d(-12, 15.84, Math.toRadians(90.0)))
                .splineToConstantHeading(new Vector2d(-12, 50), Math.toRadians(90.0))
                .build();

        // (-12, 50, 90) -> back to start
        Action backToStartFromNeg12_50 = drive.actionBuilder(
                        new Pose2d(-12, 50, Math.toRadians(90.0)))
                .splineToConstantHeading(new Vector2d(64, 15.84), Math.toRadians(-90.0))
                .build();

        // final move: go to (64, 15.84, 90) (this is basically start)
        Action finalBackToStart = drive.actionBuilder(START_POSE)
                .splineToConstantHeading(new Vector2d(64, 15.84), Math.toRadians(90.0))
                .build();

        // 9. put it all together
        Action fullAuto = new SequentialAction(
                // run launch right after start
                launchSeq,

                // 1st path
                moveTo36_1584,
                moveTo36_50,
                backToStartFrom36_50,

                // launch again
                launchSeq,

                // 2nd path
                moveTo12_1584,
                moveTo12_50,
                backToStartFrom12_50,

                // launch again
                launchSeq,

                // 3rd path
                moveToNeg12_1584,
                moveToNeg12_50,
                backToStartFromNeg12_50,

                // launch again
                launchSeq,

                // final move to (64, 15.84, 90) = end of code

                finalBackToStart
        );

        // 10. run it (blocking)
        Actions.runBlocking(fullAuto);

        // 11. telemetry loop
        while (opModeIsActive()) {
            double launcherVel = launcherMotor.getVelocity(); // ticks / s

            telemetry.addData("Launcher power", launcherMotor.getPower());
            telemetry.addData("Launcher vel (ticks/s)", launcherVel);

            telemetry.addData("Turret target", TURRET_TARGET_TICKS);
            telemetry.addData("Turret pos", turretMotor.getCurrentPosition());
            telemetry.addData("Turret busy", turretMotor.isBusy());

                       telemetry.update();

            idle();
        }
    }

    // ===== helper actions =====

    // one fire + reset:
    // trigger -> 0.9, stopper -> 0.0, wait 0.5
    // trigger -> 0.3, stopper -> 0.45, wait 0.5
    private Action buildSingleLaunchCycle() {
        return new SequentialAction(
                servoSetAction(launchTrigger, 0.9),
                servoSetAction(artifactStopper, 0.0),
                new SleepAction(0.5),
                servoSetAction(launchTrigger, 0.3),
                servoSetAction(artifactStopper, 0.45),
                new SleepAction(0.5)
        );
    }

    // you said: "then repeat this section 2 more times" -> total 3x
    private Action buildTripleLaunchAction() {
        return new SequentialAction(
                buildSingleLaunchCycle(),
                buildSingleLaunchCycle(),
                buildSingleLaunchCycle()
        );
    }

    // turn a servo set into a 1-tick action
    private Action servoSetAction(Servo servo, double position) {
        return new Action() {
            @Override
            public boolean run(TelemetryPacket packet) {
                servo.setPosition(position);
                return false; // done immediately
            }
        };
    }
}
