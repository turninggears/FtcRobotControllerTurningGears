
package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

//James For Prez


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.System.Intake;
import org.firstinspires.ftc.teamcode.System.Launcher;
import org.firstinspires.ftc.teamcode.System.MatchSettings;
import org.firstinspires.ftc.teamcode.System.Vision;

@SuppressLint("DefaultLocale")
@Config
@TeleOp(name = "CompTeleOp", group = "Test")
public class CompTeleOp extends OpMode {

    // ========================================
    // DASHBOARD - AUTOMATIC AIM SETTINGS
    // ========================================

    public static double AUTO_AIM_KP = 0.015;
    public static double AUTO_AIM_MAX_TURN = 0.35;
    public static double AUTO_AIM_MIN_TURN = 0.0;

    // Change to -1 if robot turns away
    // from the AprilTag instead of toward it.
    public static double AUTO_AIM_DIRECTION = 1.0;

    public static boolean AUTO_AIM_ENABLED = true;

    // ========================================
    // DASHBOARD - DISTANCE VELOCITY SETTINGS
    // ========================================

    public static boolean AUTO_VELOCITY_ENABLED = true;

    // Distance calibration points, inches.
    public static double DISTANCE_1 = 40;
    public static double DISTANCE_2 = 48.0;
    public static double DISTANCE_3 = 55.0;
    public static double DISTANCE_4 = 60.0;

    // Launcher Velocity Presets
    public static double VELOCITY_1 = 1180.0;
    public static double VELOCITY_2 = 1180.0;
    public static double VELOCITY_3 = 1220.0;
    public static double VELOCITY_4 = 1270.0;


    // ========================================
    // ROBOT SYSTEMS
    // ========================================

    private MecanumDriveBiobuzz drive;
    private Launcher launcher;
    private Intake intake;
    private Vision vision;

    // ========================================
    // DRIVING VARIABLES
    // ========================================

    private boolean fieldCentric = true;
    private boolean previousTriangle = false;
    private boolean previousSquare = false;
    private double headingOffset = 0.0;

    private boolean firstLoop = true;

    // ========================================
    // INTAKE VARIABLES
    // ========================================

    // 0 = stopped
    // 1 = forward
    // -1 = reverse
    private int intakeDirection = 0;

    //front intake direction tracking
    private boolean frontIntakeReversed = false;

    //tracks L2 presses for toggling
    private boolean previousFrontReverseTrigger = false;


    // ========================================
    // LAUNCHER VELOCITY VARIABLES
    // ========================================

    // Last calculated velocity, initialized to 1180.
    private double baseTargetVelocity = 1180.0;

    // Driver adjustment applied to every calculation.
    private double velocityOffset = 0.0;

    // Adjustable through FTC Dashboard.
    public static double VELOCITY_OFFSET_STEP = 10.0;

    // ========================================
    // AIMING VARIABLES
    // ========================================

    private boolean aimingRequested = false;
    private boolean aimingActive = false;

    private Double aimCorrection = null;
    private Double targetDistance = null;

    private String aimStatus = "MANUAL";

    // ========================================
    // INITIALIZATION
    // ========================================

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        drive = new MecanumDriveBiobuzz(
                hardwareMap,
                new Pose2d(0, 0, 0)
        );

        launcher = new Launcher(
                hardwareMap,
                telemetry
        );

        launcher.stopPollenLauncher();

        intake = new Intake(
                hardwareMap,
                telemetry
        );

        intake.stop();
        intake.rampsUp();

        vision = new Vision(hardwareMap);

       //Initial Launcher Velocity

        baseTargetVelocity = 1180.0;
        velocityOffset= 0.0;

        //Set Initial taget to 1180 ticks/sec
        // Set initial target to 1180 ticks/sec
        Launcher.LAUNCHER_VELOCITY = baseTargetVelocity;

        firstLoop = true;

        telemetry.addLine("BuildTeleOpV2 Initialized");
        telemetry.addData(
                "Alliance",
                MatchSettings.getAlliance()
        );
        telemetry.addLine(
                "Gamepad1 Cross = Hold to Auto Aim"
        );
        telemetry.update();
    }

    // ========================================
    // ALLIANCE SELECTION DURING INIT
    // ========================================

    @Override
    public void init_loop() {

        // D-pad LEFT selects RED
        if (gamepad1.dpad_left) {
            MatchSettings.setAlliance(
                    MatchSettings.Alliance.RED
            );
        }

        // D-pad RIGHT selects BLUE
        if (gamepad1.dpad_right) {
            MatchSettings.setAlliance(
                    MatchSettings.Alliance.BLUE
            );
        }

        telemetry.addLine("");
        telemetry.addLine("===== ALLIANCE SELECTION =====");

        telemetry.addData(
                "Selected Alliance",
                MatchSettings.getAlliance()
        );

        telemetry.addLine("D-Pad LEFT = RED");
        telemetry.addLine("D-Pad RIGHT = BLUE");
        telemetry.addLine("Press START when ready");

        telemetry.update();
    }

    // ========================================
    // MAIN LOOP
    // ========================================

    @Override
    public void loop() {

        // ====================================
        // INITIAL START
        // ====================================

        if (firstLoop) {

            intakeDirection = 1;
            frontIntakeReversed = false;
            previousFrontReverseTrigger = gamepad2.left_trigger> 0.5;
            intake.start();//starts both intake motors forward

            launcher.startPollenLauncher();//starts pollen launcher

            firstLoop = false;
        }

        // ====================================
        // UPDATE VISION
        // ====================================

        updateVisionData();

        // ====================================
        // GAMEPAD 1 - DRIVING
        // ====================================

        drive.updatePoseEstimate();

        double currentHeading =
                drive.localizer.getPose()
                        .heading.toDouble();

        // Triangle - Field/Robot Centric
        if (gamepad1.triangle && !previousTriangle) {
            fieldCentric = !fieldCentric;
        }

        previousTriangle = gamepad1.triangle;

        // Square - Reset Field Heading
        if (gamepad1.square && !previousSquare) {
            headingOffset = currentHeading;
        }

        previousSquare = gamepad1.square;

        // Left stick translation
        double forward =
                -gamepad1.left_stick_y;

        double strafe =
                -gamepad1.left_stick_x;

        // Right stick manual rotation
        double turn =
                -gamepad1.right_stick_x;

        // R1 - Half speed
        double speedMultiplier =
                gamepad1.right_bumper ? 0.5 : 1.0;

        Vector2d driveVector =
                new Vector2d(forward, strafe);

        // ====================================
        // FIELD-CENTRIC CONVERSION
        // ====================================

        if (fieldCentric) {

            double driveHeading =
                    currentHeading - headingOffset;

            driveVector =
                    Rotation2d.fromDouble(-driveHeading)
                            .times(driveVector);
        }

        // ====================================
        // GAMEPAD 1 CROSS - AUTO AIM
        // ====================================

        aimingRequested = gamepad1.cross;
        aimingActive = false;

        if (aimingRequested && AUTO_AIM_ENABLED) {

            if (aimCorrection != null) {

                turn = calculateAutoAimTurn(
                        aimCorrection
                );

                aimingActive = true;

                if (Math.abs(aimCorrection)
                        <= Vision.AIM_TOLERANCE_DEGREES) {

                    aimStatus = "ALIGNED";

                } else {

                    aimStatus = "AIMING";
                }

            } else {

                // No usable tag.
                // Manual right-stick rotation
                // remains available.
                aimStatus = "NO VALID ALLIANCE TAG";
            }

        } else if (aimingRequested) {

            aimStatus = "AUTO AIM DISABLED";

        } else {

            aimStatus = "MANUAL";
        }

        // ====================================
        // APPLY DRIVE POWERS
        // ====================================

        drive.setDrivePowers(
                new PoseVelocity2d(
                        driveVector.times(
                                speedMultiplier
                        ),
                        turn * speedMultiplier
                )
        );

        // ====================================
        // GAMEPAD 1 L1 - INTAKE RAMPS
        // ====================================

        if (gamepad1.leftBumperWasPressed()) {
            intake.rampsDown();
        }

        if (gamepad1.leftBumperWasReleased()) {
            intake.rampsUp();
        }

        // ====================================
        // GAMEPAD 2 - POLLEN GATE
        // ====================================

        // Cross - Hold gate open
        if (gamepad2.crossWasPressed()) {
            launcher.openPollenGate();
        }

        if (gamepad2.crossWasReleased()) {
            launcher.closePollenGate();
        }

        // ====================================
        // GAMEPAD 2 - LAUNCHER CONTROLS
        // ====================================

        // Circle - Forward
        // resets front intake to in
        if (gamepad2.circleWasPressed()) {

            launcher.startPollenLauncher();

            frontIntakeReversed = false;

            if (intakeDirection == 1) {
                intake.startFrontIntake();
            }
        }

        // Square - Reverse
        else if (gamepad2.squareWasPressed()) {

            launcher.reversePollenLauncher();
        }

        // Triangle - Stop
        else if (gamepad2.triangleWasPressed()) {

            launcher.stopPollenLauncher();
        }

        // D-Pad Up - Increase velocity offset by 10
        else if (gamepad2.dpadUpWasPressed()) {

            velocityOffset += VELOCITY_OFFSET_STEP;
        }

// D-Pad Down - Decrease velocity offset by 10
        else if (gamepad2.dpadDownWasPressed()) {

            velocityOffset -= VELOCITY_OFFSET_STEP;
        }

        // ====================================
        // UPDATE LAUNCHER TARGET VELOCITY
        // ====================================

        updateLauncherVelocity();

        launcher.updatePollenLauncher();

        // ====================================
        // GAMEPAD 2 - INTAKE CONTROLS
        // ====================================

        // R1 - Reverse while held
        if (gamepad2.rightBumperWasPressed()) {

            intakeDirection = -1;
            intake.reverse();
        }

        if (gamepad2.rightBumperWasReleased()) {

            intakeDirection = 1;
            intake.start();
        }

        // L1 - Toggle intake forward/stopped
        else if (gamepad2.leftBumperWasPressed()) {

            if (intakeDirection == 1) {

                intakeDirection = 0;
                intake.stop();

            } else {

                intakeDirection = 1;
                intake.start();
            }
        }

        // Gamepad 2 L2 intentionally unassigned.

        // ====================================
        // TELEMETRY
        // ====================================

        updateTelemetry();
    }

    // ========================================
// UPDATE APRILTAG DATA
// ========================================

    private void updateVisionData() {

        aimCorrection = null;
        targetDistance = null;

        if (!vision.isCameraStreaming()) {
            return;
        }

        // Get aiming correction from the
        // currently selected alliance cluster.
        aimCorrection =
                vision.getAllianceAimCorrection();

        // Get distance from pollen release
        // point to detected alliance cluster.
        targetDistance =
                vision.getLauncherDistance();

        // Update target velocity whenever
        // a valid alliance tag is detected.
        if (AUTO_VELOCITY_ENABLED
                && targetDistance != null
                && Double.isFinite(targetDistance)
                && targetDistance > 0.0) {

            baseTargetVelocity =
                    calculateVelocityForDistance(
                            targetDistance
                    );
        }

        // No valid tag:
        // Keep the previously calculated velocity.
    }

// ========================================
// CALCULATE AUTOMATIC TURN
// ========================================

    private double calculateAutoAimTurn(
            double correctionDegrees) {

        if (Math.abs(correctionDegrees)
                <= Vision.AIM_TOLERANCE_DEGREES) {

            return 0.0;
        }

        double turnPower =
                correctionDegrees * AUTO_AIM_KP;

        // Clamp turn power.
        turnPower = Math.max(
                -Math.abs(AUTO_AIM_MAX_TURN),
                Math.min(
                        Math.abs(AUTO_AIM_MAX_TURN),
                        turnPower
                )
        );

        // Optional minimum turning power.
        if (AUTO_AIM_MIN_TURN > 0.0
                && Math.abs(turnPower)
                < AUTO_AIM_MIN_TURN) {

            turnPower =
                    Math.copySign(
                            AUTO_AIM_MIN_TURN,
                            turnPower
                    );
        }

        return turnPower * AUTO_AIM_DIRECTION;
    }

    // ========================================
    // DISTANCE-BASED VELOCITY
    // ========================================

    private double calculateVelocityForDistance(
            double distance) {

        // Distance points must remain ordered:
        // DISTANCE_1 < DISTANCE_2
        // DISTANCE_2 < DISTANCE_3
        // DISTANCE_3 < DISTANCE_4

        if (distance <= DISTANCE_1) {
            return VELOCITY_1;
        }

        if (distance <= DISTANCE_2) {

            return interpolate(
                    distance,
                    DISTANCE_1,
                    DISTANCE_2,
                    VELOCITY_1,
                    VELOCITY_2
            );
        }

        if (distance <= DISTANCE_3) {

            return interpolate(
                    distance,
                    DISTANCE_2,
                    DISTANCE_3,
                    VELOCITY_2,
                    VELOCITY_3
            );
        }

        if (distance <= DISTANCE_4) {

            return interpolate(
                    distance,
                    DISTANCE_3,
                    DISTANCE_4,
                    VELOCITY_3,
                    VELOCITY_4
            );
        }

        return VELOCITY_4;
    }

    private double interpolate(
            double distance,
            double startDistance,
            double endDistance,
            double startVelocity,
            double endVelocity) {

        if (endDistance <= startDistance) {
            return startVelocity;
        }

        double fraction =
                (distance - startDistance)
                        / (endDistance - startDistance);

        return startVelocity
                + fraction
                * (endVelocity - startVelocity);
    }

    // ========================================
    // APPLY LAUNCHER VELOCITY
    // ========================================
    private void updateLauncherVelocity() {

        // Calculate the final velocity using
        // the last AprilTag calculation plus
        // the driver's offset.

        double finalTargetVelocity =
                Math.max(
                        0.0,
                        baseTargetVelocity + velocityOffset
                );

        // Send final velocity to launcher.
        Launcher.LAUNCHER_VELOCITY = finalTargetVelocity;
    }

    // ========================================
    // TELEMETRY
    // ========================================

    private void updateTelemetry() {

        telemetry.addLine("");
        telemetry.addLine("===== BIOBUZZ V2 =====");

        telemetry.addData(
                "Alliance",
                MatchSettings.getAlliance()
        );

        telemetry.addData(
                "Camera",
                vision.isCameraStreaming()
                        ? "Streaming"
                        : "Not Streaming"
        );

        telemetry.addData(
                "Detected Cluster",
                vision.getAllianceClusterName()
        );

        telemetry.addData(
                "Alliance Tag Visible",
                vision.isAllianceClusterVisible()
        );

        telemetry.addLine("");
        telemetry.addLine("===== AUTO AIM =====");

        telemetry.addData(
                "Aim Button Held",
                aimingRequested
        );

        telemetry.addData(
                "Auto Aim Active",
                aimingActive
        );

        telemetry.addData(
                "Aim Status",
                aimStatus
        );

        if (aimCorrection != null) {
            telemetry.addData(
                    "Aim Correction (deg)",
                    aimCorrection
            );
        }

        if (targetDistance != null) {
            telemetry.addData(
                    "Target Distance (in)",
                    targetDistance
            );
        }

        telemetry.addLine("");
        telemetry.addLine("===== LAUNCHER =====");


        telemetry.addData(
                "Base Target Velocity",
                baseTargetVelocity
        );

        telemetry.addData(
                "Velocity Offset",
                "%+.0f",
                velocityOffset
        );

        telemetry.addData(
                "Final Target Velocity",
                launcher.getPollenLauncherTargetVelocity()
        );

        telemetry.addData(
                "Actual Launcher Velocity",
                launcher.getPollenLauncherVelocity()
        );

        telemetry.addLine("");
        telemetry.addLine("===== DRIVING =====");

        telemetry.addData(
                "Drive Mode",
                fieldCentric
                        ? "Field Centric"
                        : "Robot Centric"
        );

        telemetry.addData(
                "Robot Heading (deg)",
                Math.toDegrees(
                        drive.localizer.getPose().heading.toDouble()
                )
        );

        telemetry.addLine("");
        telemetry.addLine("GAMEPAD 1:");
        telemetry.addLine("Cross = Hold Auto Aim");
        telemetry.addLine("Triangle = Field/Robot Centric");
        telemetry.addLine("Square = Reset Heading");
        telemetry.addLine("R1 = 50% Speed");
        telemetry.addLine("L1 = Hold Ramps Down");

        telemetry.update();
    }

    // ========================================
    // STOP - SAFETY
    // ========================================

    @Override
    public void stop() {

        if (launcher != null) {
            launcher.stopPollenLauncher();
        }

        if (drive != null) {

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(0, 0),
                            0
                    )
            );
        }

        if (intake != null) {
            intake.stop();
        }

        if (vision != null) {
            vision.close();
        }

        intakeDirection = 0;
    }
}
