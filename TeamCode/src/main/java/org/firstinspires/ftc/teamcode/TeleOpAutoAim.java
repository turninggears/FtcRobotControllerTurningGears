package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@TeleOp(name = "TeleOpAutoAim", group = "Robot")
@Config

public class TeleOpAutoAim extends OpMode {
    GoBildaPinpointDriver pinpoint;

    public static double maxSpeed = 1.0;  // make this slower for outreaches
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
    ServoController controlHubServoController;

    double launcherPower = 0;
    double launcherVelocity = 900;
    int intakeMotorMode = 0;
    double TICKS_PER_REV = 537.7;

    // This declares the IMU needed to get the current direction the robot is facing
    // TODO: change this to use the Pinpoint for localization
    IMU imu;

    // TODO: add AutoAim variable declarations here

    String alliance = "red";
    boolean firstLoop = true;
    int xGoal = -65;
    int yGoal = 0;
    double dTurret = 3.0;
    int adjustV = 0;
    double ROBOT_CENTER_X = 8.169;
    double ROBOT_CENTER_Y = 8.169;
    double startPosX = 63.831;
    double startPosY = 0;
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

    @Override
    public void init() {
        //this assigns the motors for drive chassis based on name in control hub
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL Drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR Drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BL Drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "BR Drive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakemotor");
        launcherMotor = hardwareMap.get(DcMotorEx.class,"launcher motor");
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");

        //this matches names of other motors in control hub to names created in beginning of this code
        controlHubServoController = hardwareMap.get(ServoController.class, "Control Hub");
        launchTrigger = hardwareMap.get(Servo.class,"launch trigger");
        artifactStopper = hardwareMap.get(Servo.class,"artifact stopper");

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
                        70,
                        1.5,
                        3,
                        0)
        );
        //if turret doesn't work get rid of these lines
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //if turret doesn't work get rif of previous two lines
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       /* this section is an example of creating pre set arm/motor position using encoder
        arm_down_position = 1;
        arm_mid_position = 655;
        arm_up_position = 940;
        Arm_Pickup_Position = 330;
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        */



//        controlHubServoController.pwmEnable();

        /*imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();*/

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        //pinpoint reset to zero its internal IMU and reset pose to (0,0,0)
        pinpoint.resetPosAndIMU();

        //configure pinpoint pods
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED, //X pod direction
                GoBildaPinpointDriver.EncoderDirection.FORWARD   //Y pod direction
        );
        pinpoint.setOffsets(5.709,3.465, DistanceUnit.INCH);
        //Pose2D pose = new Pose2D(DistanceUnit.INCH, (ROBOT_CENTER_X + startPosX), (ROBOT_CENTER_Y + startPosY), AngleUnit.DEGREES, 90);
        //pinpoint.setPosition(pose);
        pinpoint.setHeading(0.0, AngleUnit.DEGREES);
        pinpoint.setPosX((ROBOT_CENTER_X + startPosX),DistanceUnit.INCH);
        pinpoint.setPosY((ROBOT_CENTER_Y + startPosY),DistanceUnit.INCH);
//        pinpoint.setHeading(0.0, AngleUnit.DEGREES);
        pinpoint.update();

        telemetry.addData("pinpoint x: ", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("pinpoint y: ", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("bot angle: ", pinpoint.getHeading(AngleUnit.DEGREES));

        if (alliance.equals("red")) {
            yGoal = 65;
        } else if (alliance.equals("blue")) {
            yGoal = -65;
        } else {
            yGoal = 0;
        }
    }

    @Override
    //this is the code that runs once you press play put game play code in this section
    public void loop() {

        if(firstLoop){
            pinpoint.setHeading(0.0, AngleUnit.DEGREES);
            pinpoint.setPosY((ROBOT_CENTER_X + startPosX)*-1.0,DistanceUnit.INCH);
            pinpoint.setPosX((ROBOT_CENTER_Y + startPosY),DistanceUnit.INCH);
            firstLoop=false;
        }

        pinpoint.update();
        //update this and reactivate them if you want message to display on driver station
        double ticksPerSecond = launcherMotor.getVelocity();
        double rpm = (ticksPerSecond/TICKS_PER_REV) * 60;

        //This gives ticks of the turret motor's rotation.
        //double turretPosition = turretMotor.getCurrentPosition();
        //This converts the turretMotorPosition to an angle to the bot in degrees.
        //double turretAngle = (turretPosition % 1080)/3;

        double xBot = pinpoint.getPosY(DistanceUnit.INCH)*-1.0;
        double yBot = pinpoint.getPosX(DistanceUnit.INCH);

        double angleBotDeg = pinpoint.getHeading(AngleUnit.DEGREES);

        double xTurret = xBot - dTurret * Math.cos(Math.toRadians(angleBotDeg));
        double yTurret = yBot - dTurret * Math.sin(Math.toRadians(angleBotDeg));

        double dx = (xGoal - xTurret) *-1.0;
        double dy = yGoal - yTurret;

        double angleGoalDeg = Math.toDegrees(Math.atan2(dx, dy));      //angle to goal from X

        double angleTurretDeg_raw = angleGoalDeg - angleBotDeg;        //angle to goal from X minus bot’s angle

        double angleTurretCurr = turretMotor.getCurrentPosition() / 3.0; //current turret position in degrees (degrees=ticks/3)

        double errorTurretDeg = Math.IEEEremainder(angleTurretDeg_raw - angleTurretCurr, 360.0); //shortest path

        double turret_unwrapped = angleTurretCurr + errorTurretDeg;    //target position

        //turretMotor.setPower(1.0);
        turretMotor.setTargetPosition((int)(1080-angleTurretDeg_raw * 3));
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1.0);

        //turretMotor.setTargetPosition((int)(turret_unwrapped * 3));  //move turret to target position (ticks=degrees*3)

        double dGoal = Math.hypot(dx, dy);  //distance from goal, needed for auto-launch power

        int minV = 860;
        int maxV = 940;
        int minD = 100;
        int maxD = 138;
        double DistRatio = (maxV-minV)/(maxD-minD);

        launcherVelocity = minV + (dGoal-minD)*DistRatio + adjustV;

        telemetry.addData("xBot: ", xBot);
      //telemetry.addData("x,y: ", xBot, yBot);
        telemetry.addData("yBot: ", yBot);
      //  telemetry.addData("xTurret: ", xTurret);
      //  telemetry.addData("yTurret: ", yTurret);
        telemetry.addData("dx: ", dx);
        telemetry.addData("dy: ", dy);
        telemetry.addData("angleGoalDeg: ", angleGoalDeg);
        telemetry.addData("angleTurretDeg_raw: ", angleTurretDeg_raw);
        telemetry.addData("angleBotDeg: ", angleBotDeg);
        telemetry.addData("errorTurretDeg: ", errorTurretDeg);
        telemetry.addData("turret_unwrapped: ", turret_unwrapped);
        telemetry.addData("angleTurretCurr: ", angleTurretCurr);
        telemetry.addData("dGoal: ", dGoal);
        telemetry.addData("launcherVelocity: ", launcherVelocity);

        // ***IF THE BOT'S LOCATION IS CONFUSED, hold both bumpers and press X to reset YAW.
        if (gamepad1.cross && gamepad1.rightBumperWasPressed() && gamepad1.leftBumperWasPressed()) {
            pinpoint.setHeading(0, AngleUnit.DEGREES);
        }

        //this is start of drive code
        // --- dynamic drive speed ---
        maxSpeed = gamepad1.right_bumper ? 0.5 : 1.0;

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


        // telemetry.addData("Front Left drive power: ", frontLeftDrive.getPower());
        // telemetry.addData("Front Right drive power: ", frontRightDrive.getPower());
        // telemetry.addData("Back Left drive power: ", backLeftDrive.getPower());
        // telemetry.addData("Back Right drive power: ", backRightDrive.getPower());
//end of first drive code--

        //intake control code
        if (intakeMotorMode == 0) {
                intakeMotor.setPower(1);
                intakeMotorMode = 1;
        }
        if (gamepad2.rightBumperWasPressed()) {
            if(intakeMotor.getPower()!=0){
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

        //Launch trigger control
        if (gamepad2.cross && Math.abs(launcherVelocity - launcherMotor.getVelocity()) < 40) {
            launchTrigger.setPosition(.9);
            artifactStopper.setPosition(0);
        } else {
            launchTrigger.setPosition(0.3);
            artifactStopper.setPosition(.45);
        }

        //launcher manual control code
        if (gamepad2.triangleWasPressed()) {
            launcherVelocity = 880;
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

        //launcherMotor.setPower(Math.abs(launcherPower));
        launcherMotor.setVelocity(launcherVelocity);


        //if (gamepad2.left_trigger > 0) {
        //    turretMotor.setPower(-.30 * gamepad2.left_trigger);
        //} else if (gamepad2.right_trigger > 0) {
        //    turretMotor.setPower(.30 * gamepad2.right_trigger);
        //} else {
        //    turretMotor.setPower(0);
        //}

        telemetry.addData("launcher velocity: ", launcherMotor.getVelocity());
        //telemetry.addData("launcher power: ", launcherPower);
        //telemetry.addData("Launcher Velocity (ticks/s)", ticksPerSecond);
        //telemetry.addData("Launcher RPM", rpm);

        //telemetry.addData("turretMotor Position: ", turretPosition);
        //telemetry.addData("turret Angle: ", turretAngle);
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
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

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
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
        //telemetry.addData("speed", maxSpeed * (frontLeftPower / maxPower));
    }


}
