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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp(name = "TeleOp Experiment", group = "Robot")
@Config

public class TeleOpExperiment extends OpMode{

    GoBildaPinpointDriver pinpoint;

    public static double TARGET_X_MM = 152.4; // +X is toward the audience
    public static double TARGET_Y_mm = 3505.2; // +Y is toward the blue alliance

    // --- Turret control constants ---
// You must set these for YOUR turret. See step 4 explanations below.
    public static int    TICKS_PER_TURRET_REV = 537;   // Example for a 312RPM goBILDA motor; change to match yours
    public static double GEAR_RATIO_TURRET    = 2.0;   // motor-to-turret ratio; e.g., 2.0 if motor turns twice per 1 turret rev
    public static double TURRET_KP            = 5.0;   // simple P gain for position control (tune this)


    public static double maxSpeed = 1.0;  // make this slower for outreaches
    // This declares the four drive chassis motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    //this declares the other motors for the robot
    DcMotor intakeMotor;

    DcMotor launcherMotor;

    DcMotor turretMotor;

    //Servo launcherServo;

    ServoController controlHubServoController;

    Servo launchTrigger;
    double launcherPower = 0;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    float theta;



    //declaring drive variables
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
       //this assigns the motors and servos for intake, launch, and turret
        intakeMotor = hardwareMap.get(DcMotor.class, "intakemotor");
        launcherMotor = hardwareMap.get(DcMotor.class,"launcher motor");
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        //launcherServo = hardwareMap.get(Servo.class, "launcher servo");
        //this matches names of other motors in control hub to names created in beginning of this code
        controlHubServoController = hardwareMap.get(ServoController.class, "Control Hub");
        launchTrigger = hardwareMap.get(Servo.class, "launch trigger");
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // We need to test once chasis is done to make sure this is still correct direction for motors.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //
        // turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        //pinpoint set up
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        //pinpoint reset to zero its internal IMU and reset pose to (0,0,0)
        pinpoint.resetPosAndIMU();

        //configure pinpoint pods
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED, //X pod direction
                GoBildaPinpointDriver.EncoderDirection.FORWARD   //Y pod direction
        );
        pinpoint.setOffsets(-120,79, DistanceUnit.MM);

    }

    @Override
    //this is the code that runs once you press play put game play code in this section
    public void loop() {
        //update pinpoint every loop
        pinpoint.update();

        //reads pose: x(mm), Y (MM), Heading (radians,unnormalized)
        double robotXmm = pinpoint.getPosX(DistanceUnit.MM);
        double robotYmm = pinpoint.getPosY(DistanceUnit.MM);
        double  robotHeading = pinpoint.getHeading(AngleUnit.DEGREES);


        //not sur eif this is already added or needs to be but this converts radians to degrees
        /* NOTE: this is not needed, since the double robotHeading is initialized to the pinpoint
         * instance variable AngleUnit.DEGREES.
         */
//        double robotHeadingDeg = Math.toDegrees(robotHeading);

        telemetry.addData("Pose (mm)", "x=%.1f, y=%1f)", robotXmm, robotYmm);
        telemetry.addData("Heading (deg)", Math.toDegrees(robotHeading));

        //begin auto turret aim code
        double dx = TARGET_X_MM - robotXmm;
        double dy = TARGET_Y_mm - robotYmm;

        //angle from robot to target in field coordinates
        double targetAngleField = Math.atan2(dy, dx); //radians

        double turretAngleNeeded = targetAngleField - robotHeading;
        //normalize to [-PI, +PI] so we choose the shortest turn
        turretAngleNeeded = Math.atan2(Math.sin(turretAngleNeeded), Math.cos(turretAngleNeeded));

        //i think this needs added and was missed in chat GPT
        /*double TICKS_PER_TURRET_REV = 537.6;
        GEAR_RATIO_TURRET = 2
         */

        //radians -> turret revolutions -> encoder ticks
        double turretRevs = turretAngleNeeded / (2.0*Math.PI);
        int ticksFromZero = (int) Math.round(turretRevs * TICKS_PER_TURRET_REV * GEAR_RATIO_TURRET);

        //THIS CODE IS THE SIMPLE VERSION TO AIM TURRET WITH RUN TO POSITION
        /*turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setTargetPosition(ticksFromZero);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  */
        //this is the more advanced way to run turret to position
        int current = turretMotor.getCurrentPosition();
        int error = ticksFromZero - current;
        double power = TURRET_KP * (error/ 1000.0); //scale error so KP isn't tiny
        power = Math.max(-0.6, Math.min(0.6, power)); //clamp
        turretMotor.setPower(power);

        telemetry.addData("Turret target (deg)", Math.toDegrees(turretAngleNeeded));
        telemetry.addData("Turret ticks target", ticksFromZero);
        //end of turret auto angle code.

        //update this and reactivate them if you want message to display on driver station
        telemetry.addLine("Press cross (X) to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
//        telemetry.addLine("The left joystick sets the robot direction");
//        telemetry.addLine("Moving the right joystick left and right turns the robot");

        /* If you press the cross (X) button, then you reset the heading to be 0 degrees;
         * this is equivalent to the resetYaw method from the IMU class.
         */
        if (gamepad1.cross) {
            pinpoint.setHeading(0, AngleUnit.DEGREES);
        }

        //this is start of drive code

//         If you press the left bumper, you get a drive from the point of view of the robot
//         (much like driving an RC vehicle)
        if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        if (gamepad2.left_bumper) {
            intakeMotor.setPower(-1);
        } if (gamepad2.right_bumper) {
            intakeMotor.setPower(1);
        } if (gamepad2.dpad_down) {
            intakeMotor.setPower(0);
        }

        if (gamepad2.squareWasPressed()) {
            launcherPower = 0;
        }

        if (gamepad2.triangleWasPressed()) {
            launcherPower += 0.02;
        }

        if (gamepad2.crossWasPressed()) {
            launcherPower -= 0.02;
        }

        if (gamepad2.circleWasPressed() && launcherPower == 0) {
            launcherPower = 0.30;
        }

        if (launcherPower > 1.0) {
            launcherPower = 1.0;
        }

        telemetry.addData("launcher power: ", launcherPower);

        launcherMotor.setPower(Math.abs(launcherPower));

        //This is code to move launch trigger
        if (gamepad2.dpad_up){
            launchTrigger.setPosition(.5);
        } else {
            launchTrigger.setPosition(.30);
        }

        //This is manual turret angle movement code
        if (gamepad2.dpad_left) {
            turretMotor.setPower(1);
        } else if (gamepad2.dpad_right) {
            turretMotor.setPower(-1);
        } else {
            turretMotor.setPower(0);
        }
    }


    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
//        theta = AngleUnit.normalizeRadians(theta -
//                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Changed to get the angle from the pinpoint instead of the internal IMU.
        theta = AngleUnit.normalizeRadians(theta - pinpoint.getHeading(AngleUnit.RADIANS));

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

        double maxPower = 1.0;


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

        // shouldn't this be just maxSpeed * maxPower?
        telemetry.addData("speed", maxSpeed * (frontLeftPower / maxPower));
    }
    //end of drive code

    //start of code for manipulator (gamepad2)

    //sample of arm controls from last season to use to start coding
        /* if (gamepad2.right_trigger > 0) {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(0.3);
            ((DcMotorEx) armMotor).setVelocity(gamepad2.right_trigger + -100);
            Release_Position = armMotor.getCurrentPosition();
        } else if (gamepad2.dpad_right) {
        if (gamepad2.dpad_up) {
            armMotor.setTargetPosition(arm_mid_position);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.3);
//            if (armMotor.getCurrentPosition() > 200) {
//                wrist.setPosition(0.5);
        } else if (gamepad2.dpad_down) {
            armMotor.setTargetPosition(arm_down_position);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.3);
//            if (armMotor.getCurrentPosition() > 200) {
//                wrist.setPosition(0.5);
        }*/
    //End of Manipulator Code

}

