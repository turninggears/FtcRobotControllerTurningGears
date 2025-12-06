package org.firstinspires.ftc.teamcode;
import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@SuppressLint("DefaultLocale")
@Disabled
@TeleOp(name = "TeleOpTEST2", group = "Robot")
@Config

public class TeleOpTEST2 extends OpMode {
    public static boolean DEBUG_TELEMETRY = false;
    GoBildaPinpointDriver pinpoint;

    public static double TURN_SPEED = 0.5;
    public static double maxSpeed = 1.0;  // make this slower for outreaches
    public static double KP = 50;
    public static double KI = 0.05;
    public static double KD = 0;
    public static double KF = 14.0;

    // This declares the four drive chassis motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    DcMotor intakeMotor;
    DcMotorEx launcherMotor;
    DcMotor turretMotor;
    Servo   launchTrigger;
    Servo   artifactStopper;
    RevBlinkinLedDriver blinkin;
    ServoController controlHubServoController;

    // === Launcher gating & state (teleop) ===
    // Velocity tolerance and dwell (in encoder ticks/sec)
    private static final double LAUNCH_BAND_TICKS_PER_SEC = 35.0; // target ±1 quantization step
    private static final int   LAUNCH_IN_BAND_REQUIRED    = 4;    // consecutive in-band samples

    // Servo positions
    private static final double TRIGGER_OPEN_POS   = 0.90;
    private static final double TRIGGER_CLOSED_POS = 0.30;
    private static final double STOPPER_OPEN_POS   = 0.00;
    private static final double STOPPER_CLOSED_POS = 0.45;

    // Timing (milliseconds)
    private static final long LAUNCH_FIRING_MS  = 250;  // how long trigger stays open
    private static final long LAUNCH_RECOVER_MS = 150;  // short settle before next shot

    // Gating and FSM state
    private int     launchInBandCount    = 0;
    private boolean launchShotRequested  = false;
    private boolean lastCross            = false;

    private enum LaunchShotState { IDLE, FIRING, RECOVERING }
    private LaunchShotState launchShotState = LaunchShotState.IDLE;

    private ElapsedTime launchShotTimer = new ElapsedTime();

    // For telemetry
    private double  launchLastMeasuredVel = 0.0;
    private boolean launchLastReady       = false;

    Object headingFromAutonomous;
    Object xFromAutonomous;
    Object yFromAutonomous;

    double launcherPower = 0;
    double launcherVelocity = 900;
    int intakeMotorMode = 0;
    double TICKS_PER_REV = 537.7;

    String alliance = "red";
    boolean firstLoop = true;
    double xBot = 0.0;
    double yBot = 0.0;
    int xGoal      = -65;
    int yGoal      = 0;
    double dTurret = 3.0;
    int adjustV    = 0;
    int adjustAim  = 0;
    int angleCHEAT = 0;
    double bbx = 0.0;
    double bby = 0.0;
    double bbh = 0.0;

    //double ROBOT_CENTER_X = 8.169;
    //double ROBOT_CENTER_Y = 8.169;
    double startPosX = 62;
    double startPosY = 8;
    float theta;
    int drivemode;
    double Slow_Speed;
    double Med_Speed;
    double High_Speed;
    double Motor_Speed;
    float vertical;
    double pivot;
    float horizontal;
    Orientation angles;
    Acceleration gravity;

    NormalizedColorSensor colorSensor;
    float colorGain = 4;
    final float[] hsvValues = new float[3];

    @Override
    public void init() {
        //this assigns the motors for drive chassis based on name in control hub
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "FL Drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR Drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "BL Drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "BR Drive");
        intakeMotor     = hardwareMap.get(DcMotor.class, "intakemotor");
        launcherMotor   = hardwareMap.get(DcMotorEx.class,"launcher motor");
        turretMotor     = hardwareMap.get(DcMotor.class, "turretMotor");

        //this matches names of other motors in control hub to names created in beginning of this code
        controlHubServoController = hardwareMap.get(ServoController.class, "Control Hub");
        launchTrigger = hardwareMap.get(Servo.class,"launch trigger");
        artifactStopper = hardwareMap.get(Servo.class,"artifact stopper");
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("=== TeleOpTEST2 ===");

        // We need to test once chasis is done to make sure this is still correct direction for motors.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        KP,
                        KI,
                        KD,
                        KF)
        );
        // turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        String xFromAutonomous = "x";
        String yFromAutonomous = "y";
        String headingFromAutonomous = "heading";

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        //pinpoint reset to zero its internal IMU and reset pose to (0,0,0)
        //testing without this line.....  pinpoint.resetPosAndIMU();
        //configure pinpoint pods
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED, //X pod direction
                GoBildaPinpointDriver.EncoderDirection.FORWARD   //Y pod direction
        );
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-5.709,3.465, DistanceUnit.INCH);
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        if (blackboard.containsKey(xFromAutonomous) && blackboard.containsKey(yFromAutonomous) && blackboard.containsKey(headingFromAutonomous)) {
            alliance = (String) blackboard.get("team");
            bbh = (double)(blackboard.get(headingFromAutonomous));
            if(alliance.equals("red")){
                bbx = (double)(blackboard.get(yFromAutonomous));
                bby = (double)(blackboard.get(xFromAutonomous))*-1.0;
            } else {
                bbx = (double)(blackboard.get(xFromAutonomous));
                bby = (double)(blackboard.get(yFromAutonomous))*-1.0;
            }
            blackboard.clear();;
        }

        pinpoint.update();

        telemetry.addData("PP x: ", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("PP y: ", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("PP angle: ",  pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("bbx: ", bbx);
        telemetry.addData("bby: ", bby);
        telemetry.addData("bbh: ", bbh);

        telemetry.addData("alliance: ", alliance);
    }

    @Override
    //This is the code that runs repeatedly once you press play. Put game play code in this section
    public void loop() {

        if(firstLoop){
            firstLoop=false;

            if(bbx+bby+bbh != 0){  //if (blackboard.containsKey(xFromAutonomous) && blackboard.containsKey(yFromAutonomous) && blackboard.containsKey(headingFromAutonomous)) {

                if(alliance.equals("red")){
                    pinpoint.setHeading(0, AngleUnit.DEGREES);
                    pinpoint.setPosY(bby, DistanceUnit.INCH);
                    pinpoint.setPosX(bbx, DistanceUnit.INCH);
                    angleCHEAT = 0;
                    adjustAim = -7;
                    adjustV = 0;
                } else {  //Alliance is BLUE
                    pinpoint.setHeading(0, AngleUnit.DEGREES);
                    pinpoint.setPosY(bbx, DistanceUnit.INCH);
                    pinpoint.setPosX(bby, DistanceUnit.INCH);
                    angleCHEAT = 0;
                    adjustAim = -7;
                    adjustV = -20;
                }

            } else {
                pinpoint.setHeading(0.0, AngleUnit.DEGREES);
                pinpoint.setPosY((startPosX)*-1.0, DistanceUnit.INCH);
                pinpoint.setPosX((startPosY), DistanceUnit.INCH);
            }

            if(alliance.equals("red")){
                yGoal = 65;
            } else {
                yGoal = -65;
            }

        }

        pinpoint.update();

        if(alliance.equals("red")){
            xBot = pinpoint.getPosY(DistanceUnit.INCH)*-1.0;
            yBot = pinpoint.getPosX(DistanceUnit.INCH);
        } else {
            xBot = pinpoint.getPosY(DistanceUnit.INCH);
            yBot = pinpoint.getPosX(DistanceUnit.INCH)*-1.0;
        }

        double angleBotDeg = pinpoint.getHeading(AngleUnit.DEGREES) ;
        double xTurret = xBot - dTurret * Math.cos(Math.toRadians(angleBotDeg+90));
        double yTurret = yBot - dTurret * Math.sin(Math.toRadians(angleBotDeg+90));
        double dx = xGoal - xTurret;
        double dy = yGoal - yTurret;

        double angleGoalDeg = Math.toDegrees(Math.atan2(dy, dx));      //angle to goal from X
        double angleTurretDeg_raw = 0.0;

        if(alliance.equals("red")){
            angleTurretDeg_raw = angleGoalDeg - angleBotDeg - 90 + angleCHEAT;        //angle to goal from X minus bot’s angle
        } else {
            angleTurretDeg_raw = angleGoalDeg - angleBotDeg + 90 + angleCHEAT;        //angle to goal from X minus bot’s angle
        }
        //double angleTurretCurr = turretMotor.getCurrentPosition() / 3.0; //current turret position in degrees (degrees=ticks/3)
        //double errorTurretDeg = Math.IEEEremainder(angleTurretDeg_raw - angleTurretCurr, 360.0); //shortest path
        //double turret_unwrapped = angleTurretCurr + errorTurretDeg;    //target position

        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setTargetPosition((int)(1080-angleTurretDeg_raw * 3) + adjustAim);
        turretMotor.setPower(1.0);

        //turretMotor.setTargetPosition((int)(turret_unwrapped * 3));  //move turret to target position (ticks=degrees*3)

        double dGoal = Math.hypot(dx, dy);  //distance from goal, needed for auto-launch power

        int minV = 860;
        int maxV = 960;
        int minD = 96;
        int maxD = 140;
        double DistRatio = (double)(maxV-minV)/(maxD-minD);

        if(launcherVelocity > 0)
        {launcherVelocity = minV + (dGoal-minD)*DistRatio + adjustV;}

        // ***IF THE BOT'S LOCATION IS CONFUSED, hold both bumpers and press X to reset YAW.
        if (gamepad1.cross && gamepad1.rightBumperWasPressed() && gamepad1.leftBumperWasPressed()) {
            pinpoint.setHeading(0, AngleUnit.DEGREES);
        }

        //this is start of drive code
        // --- dynamic drive speed ---
        maxSpeed = gamepad1.right_bumper ? 0.3 : 1.0;

        // --- drive control ---
        if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

//         If you press the left bumper, you get a drive from the point of view of the robot
//         (much like driving an RC vehicle)
        if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

//end of first drive code--

        //intake control code
        if (intakeMotorMode == 0) {
                intakeMotor.setPower(1);
                intakeMotorMode = 1;
        }
        if (gamepad2.rightBumperWasPressed()) {
            if( intakeMotor.getPower()!=0 ){
                intakeMotor.setPower(0);
            } else {
                intakeMotor.setPower(1);
            }
        }

        if (gamepad2.left_bumper) {
            if (intakeMotor.getPower() != 0) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(1);
            }
        } else if(intakeMotor.getPower()!=0){
            intakeMotor.setPower(1);
        }

        // === Launch trigger control: gated + no-chatter ===

        // 1) Compute "readyToShoot" from the current target & measured velocity
        double currentLaunchVel = launcherMotor.getVelocity();
        boolean inBand = Math.abs(launcherVelocity - currentLaunchVel) < LAUNCH_BAND_TICKS_PER_SEC;

        if (inBand) {
            launchInBandCount++;
        } else {
            launchInBandCount = 0;
        }

        boolean readyToShoot = (launchInBandCount >= LAUNCH_IN_BAND_REQUIRED);

        // Store for telemetry
        launchLastMeasuredVel = currentLaunchVel;
        launchLastReady       = readyToShoot;

        // 2) Edge-detect CROSS on gamepad2 and queue a shot
        boolean crossNow = gamepad2.cross;
        boolean crossPressed = crossNow && !lastCross;
        lastCross = crossNow;

        if (crossPressed) {
            launchShotRequested = true;   // one queued shot per press
        }

        // 3) Launcher state machine (servo control)
        switch (launchShotState) {
            case IDLE:
                // Default safe positions
                launchTrigger.setPosition(TRIGGER_CLOSED_POS);
                artifactStopper.setPosition(STOPPER_CLOSED_POS);

                // Fire only when queued AND we've been at speed for multiple loops
                if (launchShotRequested && readyToShoot) {
                    launchTrigger.setPosition(TRIGGER_OPEN_POS);
                    artifactStopper.setPosition(STOPPER_OPEN_POS);

                    launchShotTimer.reset();
                    launchShotState = LaunchShotState.FIRING;
                    launchShotRequested = false;  // consume request
                }
                break;

            case FIRING:
                // Hold trigger open for a fixed time
                if (launchShotTimer.milliseconds() < LAUNCH_FIRING_MS) {
                    launchTrigger.setPosition(TRIGGER_OPEN_POS);
                    artifactStopper.setPosition(STOPPER_OPEN_POS);
                } else {
                    // Close back up
                    launchTrigger.setPosition(TRIGGER_CLOSED_POS);
                    artifactStopper.setPosition(STOPPER_CLOSED_POS);

                    launchShotTimer.reset();
                    launchShotState = LaunchShotState.RECOVERING;
                }
                break;

            case RECOVERING:
                // Short settle delay before allowing the next shot
                if (launchShotTimer.milliseconds() > LAUNCH_RECOVER_MS) {
                    launchShotState = LaunchShotState.IDLE;
                }
                break;
        }

        //if (gamepad1.cross && Math.abs(launcherVelocity - launcherMotor.getVelocity()) < 35) {
            //launchTrigger.setPosition(.9);
            //artifactStopper.setPosition(0);
        //} else {
            //launchTrigger.setPosition(0.3);
            //artifactStopper.setPosition(.45);
        //}

        //launcher manual control code
        if (gamepad2.triangleWasPressed()) {
            if(alliance.equals("red"))
              {alliance="blue";
               blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
               yGoal = -65;}
            else
              {alliance="red";
               blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
               yGoal = 65;
              }

        } else if (gamepad2.squareWasPressed()) {
            launcherVelocity = 0;
        } else if (gamepad2.circleWasPressed()) {
            launcherVelocity = 960;
        }

        if (gamepad2.dpadUpWasPressed()) {
            adjustV += 20;
        }

        if (gamepad2.dpadDownWasPressed()) {
            adjustV -= 20;
        }

        if (launcherVelocity > 1200) {
            launcherVelocity = 1200;
        }

        launcherMotor.setVelocity(launcherVelocity);

        if (gamepad2.left_trigger > 0) {
            adjustAim = adjustAim - 1;
        } else if (gamepad2.right_trigger > 0) {
            adjustAim = adjustAim + 1;
        }


        colorSensor.setGain(colorGain);
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double rrr = colors.red   * 255.0;
        double ggg = colors.green * 255.0;
        double bbb = colors.blue  * 255.0;
        double avgColor = (rrr+ggg+bbb)/3.0;
        boolean isEmpty;
        if(avgColor > 2.2){
            isEmpty = false;
            if ("red".equals(alliance)) {   // safe against nulls too
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else {
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }
        } else {
            isEmpty = true;
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }

        // --- Basic telemetry (always on) ---
        telemetry.addData("pinpoint x (in)", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("pinpoint y (in)", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("heading (deg)",   pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("alliance",        alliance);

        telemetry.addData("launcher target", "%.0f", launcherVelocity);
        telemetry.addData("launcher vel",    "%.0f", launchLastMeasuredVel);
        telemetry.addData("launch state",    launchShotState);
        telemetry.addData("launch ready",    launchLastReady);

        telemetry.addData("isEmpty", isEmpty);

//telemetry.addData("errorTurretDeg: ", errorTurretDeg);
//telemetry.addData("turret_unwrapped: ", turret_unwrapped);
//telemetry.addData("angleTurretCurr: ", angleTurretCurr);

        // --- Debug telemetry (toggle in Dashboard) ---
        if (DEBUG_TELEMETRY) {
            telemetry.addData("timestamp", System.nanoTime());
            telemetry.addData("bbx", bbx);
            telemetry.addData("bby", bby);
            telemetry.addData("bbh", bbh);

            telemetry.addData("xBot",    xBot);
            telemetry.addData("yBot",    yBot);
            telemetry.addData("xTurret", xTurret);
            telemetry.addData("yTurret", yTurret);
            telemetry.addData("turret enc", turretMotor.getCurrentPosition());
            telemetry.addData("adjustAim", adjustAim);
            telemetry.addData("adjustV",   adjustV);

            telemetry.addData("dx",  dx);
            telemetry.addData("dy",  dy);
            telemetry.addData("dGoal", dGoal);
            telemetry.addData("angGoal", angleGoalDeg);
            telemetry.addData("angTurretRaw", angleTurretDeg_raw);
            telemetry.addData("angBot", angleBotDeg);

            telemetry.addData("launch inBand", launchInBandCount);
            telemetry.addData("launch queued", launchShotRequested);
            telemetry.addData("inBand", launchInBandCount);

            telemetry.addData("launcherVelRaw", launcherMotor.getVelocity());
            telemetry.addData("error", launcherVelocity - launcherMotor.getVelocity());

            telemetry.addData("Red",   rrr);
            telemetry.addData("Green", ggg);
            telemetry.addData("Blue",  bbb);
        }
        telemetry.update();
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight =   r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower  = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower  = forward + right - rotate;
        double backLeftPower   = forward - right + rotate;

        double maxPower = 1;

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed  * (frontLeftPower  / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed   * (backLeftPower   / maxPower));
        backRightDrive.setPower(maxSpeed  * (backRightPower  / maxPower));
        //telemetry.addData("speed", maxSpeed * (frontLeftPower / maxPower));
    }

}