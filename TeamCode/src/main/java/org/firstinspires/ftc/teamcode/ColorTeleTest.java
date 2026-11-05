
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
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.System.ColorSensors;
import org.firstinspires.ftc.teamcode.System.Intake;
import org.firstinspires.ftc.teamcode.System.Launcher;
import org.firstinspires.ftc.teamcode.System.MatchSettings;
import org.firstinspires.ftc.teamcode.System.Vision;

@SuppressLint("DefaultLocale")
@Config
@TeleOp(name = "ColorTeleTest", group = "Test")
public class ColorTeleTest extends OpMode {

    // ========================================
    // DASHBOARD - AUTOMATIC AIM SETTINGS
    // ========================================

    public static double AUTO_AIM_KP = 0.015;
    public static double AUTO_AIM_MAX_TURN = 0.35;
    public static double AUTO_AIM_MIN_TURN = 0.0;
    public static double AUTO_AIM_DIRECTION = 1.0;
    public static boolean AUTO_AIM_ENABLED = true;

    // ========================================
    // DASHBOARD - DISTANCE VELOCITY SETTINGS
    // ========================================

    public static boolean AUTO_VELOCITY_ENABLED = true;

    public static double DISTANCE_1 = 40;
    public static double DISTANCE_2 = 48.0;
    public static double DISTANCE_3 = 55.0;
    public static double DISTANCE_4 = 60.0;

    public static double VELOCITY_1 = 1190.0;
    public static double VELOCITY_2 = 1190.0;
    public static double VELOCITY_3 = 1220.0;
    public static double VELOCITY_4 = 1270.0;

    public static double VELOCITY_OFFSET_STEP = 10.0;

    // ========================================
    // DASHBOARD - COLOR SENSOR SETTINGS
    // ========================================

    public static boolean AUTO_REJECT_ENABLED = true;

    // Reverse both motors until this much time
    // has passed since the LAST nectar detection.
    public static double NECTAR_CLEAR_SECONDS = 0.75;

    // Debounce full and open conditions.
    public static double FULL_DEBOUNCE_SECONDS = 0.15;
    public static double OPEN_DEBOUNCE_SECONDS = 0.15;

    // Tune using actual sensor readings.
    public static double BALL_ALPHA_THRESHOLD = 0.800;

    public static double RED_HUE_MAX = 40.0;
    public static double RED_HUE_HIGH_MIN = 340.0;

    public static double YELLOW_HUE_MIN = 70.0;
    public static double YELLOW_HUE_MAX = 120.0;

    public static double BLUE_HUE_MIN = 180.0;
    public static double BLUE_HUE_MAX = 260.0;

    // ========================================
    // ROBOT SYSTEMS
    // ========================================

    private MecanumDriveBiobuzz drive;
    private Launcher launcher;
    private Intake intake;
    private Vision vision;
    private ColorSensors normalizedColorSensors;

    // ========================================
    // DRIVING VARIABLES
    // ========================================

    private boolean fieldCentric = true;
    private boolean previousTriangle = false;
    private boolean previousSquare = false;
    private double headingOffset = 0.0;
    private boolean firstLoop = true;

    // ========================================
    // MANUAL INTAKE VARIABLES
    // ========================================

    // 0 = stopped, 1 = forward, -1 = reverse
    private int intakeDirection = 0;
    private boolean frontIntakeReversed = false;
    private boolean previousFrontReverseTrigger = false;

    // ========================================
    // LAUNCHER VARIABLES
    // ========================================

    private double baseTargetVelocity = 1190.0;
    private double velocityOffset = 0.0;

    // ========================================
    // AIMING VARIABLES
    // ========================================

    private boolean aimingRequested = false;
    private boolean aimingActive = false;

    private Double aimCorrection = null;
    private Double targetDistance = null;

    private String aimStatus = "MANUAL";

    // ========================================
    // AUTOMATIC INTAKE VARIABLES
    // ========================================

    // Gamepad 2 Options toggles AUTO / MANUAL.
    private boolean automaticIntakeEnabled = true;

    // L1 toggles the entire intake ON / OFF.
    private boolean intakeEnabled = true;

    private boolean previousOptions = false;

    private boolean sensorFault = false;
    private String sensorFaultMessage = "";

    private String intakeStatus = "NORMAL";

    private int commandedMain = 0;
    private int commandedFront = 0;

    private int pollenCount = 0;
    private boolean rawFull = false;
    private boolean fullConfirmed = false;

    private double fullSince = -1.0;
    private double openSince = -1.0;

    private boolean nectarRejectionActive = false;
    private double lastNectarSeenTime = -1.0;

    private final String[] sensorColors = new String[4];
    private final double[] sensorHues = new double[4];
    private final double[] sensorAlphas = new double[4];

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

        launcher = new Launcher(hardwareMap, telemetry);
        launcher.stopPollenLauncher();

        intake = new Intake(hardwareMap, telemetry);
        intake.stop();
        intake.rampsUp();

        vision = new Vision(hardwareMap);

        normalizedColorSensors = new ColorSensors(hardwareMap);

        baseTargetVelocity = 1180.0;
        velocityOffset = 0.0;

        Launcher.LAUNCHER_VELOCITY = baseTargetVelocity;

        firstLoop = true;
        automaticIntakeEnabled = AUTO_REJECT_ENABLED;
        intakeEnabled = true;
        previousOptions = gamepad2.options;

        telemetry.addLine("ColorTeleTest Initialized");
        telemetry.addData("Alliance", MatchSettings.getAlliance());
        telemetry.addLine("Gamepad1 Cross = Hold to Auto Aim");
        telemetry.addLine("Gamepad2 Options = Auto/Manual Intake");
        telemetry.addLine("Gamepad2 L1 = Intake On/Off");
        telemetry.update();
    }

    // ========================================
    // ALLIANCE SELECTION DURING INIT
    // ========================================

    @Override
    public void init_loop() {

        if (gamepad1.dpad_left) {
            MatchSettings.setAlliance(MatchSettings.Alliance.RED);
        }

        if (gamepad1.dpad_right) {
            MatchSettings.setAlliance(MatchSettings.Alliance.BLUE);
        }

        telemetry.addLine("===== ALLIANCE SELECTION =====");
        telemetry.addData("Selected Alliance", MatchSettings.getAlliance());
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

        if (firstLoop) {

            intakeDirection = 1;
            intakeEnabled = true;
            frontIntakeReversed = false;

            previousFrontReverseTrigger =
                    gamepad2.left_trigger > 0.5;

            previousOptions = gamepad2.options;

            launcher.startPollenLauncher();

            firstLoop = false;
        }

        // ====================================
        // UPDATE VISION
        // ========================================

        updateVisionData();

        // ====================================
        // GAMEPAD 1 - DRIVING
        // ========================================

        drive.updatePoseEstimate();

        double currentHeading =
                drive.localizer.getPose().heading.toDouble();

        // Triangle: Field/Robot centric toggle.
        if (gamepad1.triangle && !previousTriangle) {
            fieldCentric = !fieldCentric;
        }
        previousTriangle = gamepad1.triangle;

        // Square: Reset field heading.
        if (gamepad1.square && !previousSquare) {
            headingOffset = currentHeading;
        }
        previousSquare = gamepad1.square;

        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // R1: Half speed.
        double speedMultiplier =
                gamepad1.right_bumper ? 0.5 : 1.0;

        Vector2d driveVector = new Vector2d(forward, strafe);

        if (fieldCentric) {
            double driveHeading = currentHeading - headingOffset;

            driveVector = Rotation2d.fromDouble(-driveHeading)
                    .times(driveVector);
        }

        // ====================================
        // GAMEPAD 1 CROSS - AUTO AIM
        // ========================================

        aimingRequested = gamepad1.cross;
        aimingActive = false;

        if (aimingRequested && AUTO_AIM_ENABLED) {

            if (aimCorrection != null) {

                turn = calculateAutoAimTurn(aimCorrection);
                aimingActive = true;

                if (Math.abs(aimCorrection)
                        <= Vision.AIM_TOLERANCE_DEGREES) {
                    aimStatus = "ALIGNED";
                } else {
                    aimStatus = "AIMING";
                }

            } else {
                aimStatus = "NO VALID ALLIANCE TAG";
            }

        } else if (aimingRequested) {
            aimStatus = "AUTO AIM DISABLED";
        } else {
            aimStatus = "MANUAL";
        }

        drive.setDrivePowers(
                new PoseVelocity2d(
                        driveVector.times(speedMultiplier),
                        turn * speedMultiplier
                )
        );

        // ====================================
        // GAMEPAD 1 L1 - INTAKE RAMPS
        // ========================================

        if (gamepad1.leftBumperWasPressed()) {
            intake.rampsDown();
        }

        if (gamepad1.leftBumperWasReleased()) {
            intake.rampsUp();
        }

        // ====================================
        // GAMEPAD 2 CROSS - POLLEN GATE
        // ========================================

        if (gamepad2.crossWasPressed()) {

            launcher.openPollenGate();

            // Only reset front intake direction
            // when sensor automation is disabled.
            if (!automaticIntakeEnabled || !AUTO_REJECT_ENABLED) {
                frontIntakeReversed = false;
            }
        }

        if (gamepad2.crossWasReleased()) {
            launcher.closePollenGate();
        }

        // ====================================
        // GAMEPAD 2 - LAUNCHER CONTROLS
        // ========================================

        if (gamepad2.circleWasPressed()) {

            launcher.startPollenLauncher();

            if (!automaticIntakeEnabled || !AUTO_REJECT_ENABLED) {
                frontIntakeReversed = false;
            }

        } else if (gamepad2.squareWasPressed()) {

            launcher.reversePollenLauncher();

        } else if (gamepad2.triangleWasPressed()) {

            launcher.stopPollenLauncher();

        } else if (gamepad2.dpadUpWasPressed()) {

            velocityOffset += VELOCITY_OFFSET_STEP;

        } else if (gamepad2.dpadDownWasPressed()) {

            velocityOffset -= VELOCITY_OFFSET_STEP;
        }

        updateLauncherVelocity();
        launcher.updatePollenLauncher();

        // ====================================
        // GAMEPAD 2 - INTAKE CONTROLS
        // ========================================

        // Options: Toggle automatic/manual mode.
        if (gamepad2.options && !previousOptions) {

            automaticIntakeEnabled = !automaticIntakeEnabled;

            nectarRejectionActive = false;
            lastNectarSeenTime = -1.0;

            fullConfirmed = false;
            fullSince = -1.0;
            openSince = -1.0;

            frontIntakeReversed = false;
            intakeDirection = intakeEnabled ? 1 : 0;
        }

        previousOptions = gamepad2.options;

        // L1: Intake ON/OFF in both modes.
        if (gamepad2.leftBumperWasPressed()) {

            intakeEnabled = !intakeEnabled;
            intakeDirection = intakeEnabled ? 1 : 0;

            if (!intakeEnabled) {
                nectarRejectionActive = false;
                lastNectarSeenTime = -1.0;
            }

            frontIntakeReversed = false;
        }

        // L2: Front intake toggle in manual mode.
        boolean frontReverseTrigger =
                gamepad2.left_trigger > 0.5;

        if ((!automaticIntakeEnabled || !AUTO_REJECT_ENABLED)
                && intakeEnabled
                && frontReverseTrigger
                && !previousFrontReverseTrigger) {

            frontIntakeReversed = !frontIntakeReversed;
        }

        previousFrontReverseTrigger = frontReverseTrigger;

        // Apply exactly one pair of motor commands
        // after processing all controls and sensors.
        updateIntakeControl();

        updateTelemetry();
    }

    // ========================================
    // AUTOMATIC INTAKE CONTROL
    // ========================================

    private void updateIntakeControl() {

        double now = getRuntime();

        if (automaticIntakeEnabled && AUTO_REJECT_ENABLED) {

            readColorSensors();

            if (sensorFault) {

                // Stop intake on sensor read failure.
                // Driver must explicitly press L1
                // before restarting in manual mode.
                automaticIntakeEnabled = false;
                intakeEnabled = false;
                intakeDirection = 0;
                nectarRejectionActive = false;
            }
        }

        int main = 0;
        int front = 0;

        if (!intakeEnabled) {

            intakeStatus = "OFF";
            nectarRejectionActive = false;
            lastNectarSeenTime = -1.0;

        } else if (!automaticIntakeEnabled || !AUTO_REJECT_ENABLED) {

            // Manual mode:
            // R1 reverses both while held.
            // L2 toggles front direction independently.
            main = gamepad2.right_bumper ? -1 : 1;

            front = gamepad2.right_bumper
                    ? -1
                    : (frontIntakeReversed ? -1 : 1);

            intakeStatus = sensorFault
                    ? "MANUAL - SENSOR FAULT"
                    : "MANUAL";

        } else {

            boolean nectarSeen = false;
            pollenCount = 0;

            for (String color : sensorColors) {

                if ("yellow".equals(color)) {
                    pollenCount++;
                }

                if ("red".equals(color)
                        || "blue".equals(color)) {
                    nectarSeen = true;
                }
            }

            // ====================================
            // DETECT FOUR POLLEN - FULL
            // ====================================

            rawFull = pollenCount == 4;

            if (rawFull) {

                openSince = -1.0;

                if (fullSince < 0) {
                    fullSince = now;
                }

                if (now - fullSince
                        >= Math.max(0, FULL_DEBOUNCE_SECONDS)) {
                    fullConfirmed = true;
                }

            } else {

                fullSince = -1.0;

                if (openSince < 0) {
                    openSince = now;
                }

                if (now - openSince
                        >= Math.max(0, OPEN_DEBOUNCE_SECONDS)) {
                    fullConfirmed = false;
                }
            }

            // ====================================
            // DETECT NECTAR
            // ====================================

            if (nectarSeen) {
                nectarRejectionActive = true;
                lastNectarSeenTime = now;
            }

            // Timer resets every time nectar is seen.
            if (nectarRejectionActive
                    && !nectarSeen
                    && now - lastNectarSeenTime
                    >= Math.max(0, NECTAR_CLEAR_SECONDS)) {

                nectarRejectionActive = false;
            }

            // ====================================
            // MOTOR PRIORITY
            // ====================================

            if (nectarRejectionActive) {

                main = -1;
                front = -1;
                intakeStatus = "REJECT NECTAR";

            } else if (fullConfirmed) {

                main = 1;
                front = -1;
                intakeStatus = "FULL - FRONT REVERSE";

            } else {

                main = 1;
                front = 1;
                intakeStatus = "AUTO INTAKE";
            }
        }

        commandedMain = main;
        commandedFront = front;

        // Requires method added to Intake.java.
        intake.setIntakeDirections(main, front);
    }

    // ========================================
    // READ ALL FOUR COLOR SENSORS
    // ========================================

    private void readColorSensors() {

        pollenCount = 0;
        sensorFault = false;
        sensorFaultMessage = "";

        for (int i = 0; i < 4; i++) {

            try {

                NormalizedRGBA colors =
                        normalizedColorSensors.getNormalizedColors(i);

                double hue =
                        normalizedColorSensors.calculateHue(colors);

                sensorHues[i] = hue;
                sensorAlphas[i] = colors.alpha;

                if (!Double.isFinite(colors.alpha)
                        || !Double.isFinite(hue)) {
                    throw new IllegalStateException("invalid reading");
                }

                String color = "empty";

                if (colors.alpha >= BALL_ALPHA_THRESHOLD) {

                    if (hue <= RED_HUE_MAX
                            || hue >= RED_HUE_HIGH_MIN) {

                        color = "red";

                    } else if (hue >= YELLOW_HUE_MIN
                            && hue <= YELLOW_HUE_MAX) {

                        color = "yellow";

                    } else if (hue >= BLUE_HUE_MIN
                            && hue <= BLUE_HUE_MAX) {

                        color = "blue";

                    } else {

                        color = "unknown";
                    }
                }

                sensorColors[i] = color;

                if ("yellow".equals(color)) {
                    pollenCount++;
                }

            } catch (Exception e) {

                sensorColors[i] = "FAULT";
                sensorHues[i] = Double.NaN;
                sensorAlphas[i] = Double.NaN;

                sensorFault = true;

                sensorFaultMessage =
                        "Sensor " + i + ": " + e.getMessage();
            }
        }
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

        aimCorrection = vision.getAllianceAimCorrection();
        targetDistance = vision.getLauncherDistance();

        if (AUTO_VELOCITY_ENABLED
                && targetDistance != null
                && Double.isFinite(targetDistance)
                && targetDistance > 0.0) {

            baseTargetVelocity =
                    calculateVelocityForDistance(targetDistance);
        }
    }

    // ========================================
    // CALCULATE AUTOMATIC TURN
    // ========================================

    private double calculateAutoAimTurn(double correctionDegrees) {

        if (Math.abs(correctionDegrees)
                <= Vision.AIM_TOLERANCE_DEGREES) {
            return 0.0;
        }

        double turnPower = correctionDegrees * AUTO_AIM_KP;

        turnPower = Math.max(
                -Math.abs(AUTO_AIM_MAX_TURN),
                Math.min(
                        Math.abs(AUTO_AIM_MAX_TURN),
                        turnPower
                )
        );

        if (AUTO_AIM_MIN_TURN > 0.0
                && Math.abs(turnPower) < AUTO_AIM_MIN_TURN) {

            turnPower = Math.copySign(
                    AUTO_AIM_MIN_TURN,
                    turnPower
            );
        }

        return turnPower * AUTO_AIM_DIRECTION;
    }

    // ========================================
    // DISTANCE-BASED VELOCITY
    // ========================================

    private double calculateVelocityForDistance(double distance) {

        if (distance <= DISTANCE_1) {
            return VELOCITY_1;
        }

        if (distance <= DISTANCE_2) {
            return interpolate(
                    distance,
                    DISTANCE_1, DISTANCE_2,
                    VELOCITY_1, VELOCITY_2
            );
        }

        if (distance <= DISTANCE_3) {
            return interpolate(
                    distance,
                    DISTANCE_2, DISTANCE_3,
                    VELOCITY_2, VELOCITY_3
            );
        }

        if (distance <= DISTANCE_4) {
            return interpolate(
                    distance,
                    DISTANCE_3, DISTANCE_4,
                    VELOCITY_3, VELOCITY_4
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
                + fraction * (endVelocity - startVelocity);
    }

    // ========================================
    // UPDATE LAUNCHER TARGET VELOCITY
    // ========================================

    private void updateLauncherVelocity() {

        double finalTargetVelocity = Math.max(
                0.0,
                baseTargetVelocity + velocityOffset
        );

        Launcher.LAUNCHER_VELOCITY = finalTargetVelocity;
    }

    // ========================================
    // TELEMETRY
    // ========================================

    private void updateTelemetry() {

        telemetry.addLine("");
        telemetry.addLine("===== BIOBUZZ V2 =====");

        telemetry.addData("Alliance", MatchSettings.getAlliance());

        telemetry.addData(
                "Camera",
                vision.isCameraStreaming()
                        ? "Streaming" : "Not Streaming"
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

        telemetry.addData("Aim Button Held", aimingRequested);
        telemetry.addData("Auto Aim Active", aimingActive);
        telemetry.addData("Aim Status", aimStatus);

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
        telemetry.addLine("===== AUTOMATIC INTAKE =====");

        telemetry.addData(
                "Mode",
                automaticIntakeEnabled && AUTO_REJECT_ENABLED
                        ? "AUTO" : "MANUAL"
        );

        telemetry.addData(
                "Intake",
                intakeEnabled ? "ON" : "OFF"
        );

        telemetry.addData("Status", intakeStatus);
        telemetry.addData("Pollen detected", pollenCount);
        telemetry.addData("Full confirmed", fullConfirmed);
        telemetry.addData("Nectar reject active", nectarRejectionActive);

        telemetry.addData(
                "Main / Front direction",
                "%d / %d",
                commandedMain,
                commandedFront
        );

        telemetry.addData(
                "Sensor fault",
                sensorFaultMessage.isEmpty()
                        ? "None" : sensorFaultMessage
        );

        for (int i = 0; i < 4; i++) {

            telemetry.addData(
                    "Sensor " + i,
                    "%s  hue=%.1f  alpha=%.3f",
                    sensorColors[i] == null
                            ? "unread" : sensorColors[i],
                    sensorHues[i],
                    sensorAlphas[i]
            );
        }

        telemetry.addLine(
                "G2 Options: Auto/Manual | G2 L1: Intake On/Off"
        );

        telemetry.addLine(
                "Manual: G2 R1 both reverse | G2 L2 front toggle"
        );

        telemetry.addLine("");
        telemetry.addLine("===== DRIVING =====");

        telemetry.addData(
                "Drive Mode",
                fieldCentric
                        ? "Field Centric" : "Robot Centric"
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
