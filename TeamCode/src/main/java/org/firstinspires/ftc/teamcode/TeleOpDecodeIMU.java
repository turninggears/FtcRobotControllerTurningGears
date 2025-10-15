package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "TeleOpDecodeIMU", group = "Robot")
@Config

public class TeleOpDecodeIMU extends OpMode {

    public static double maxSpeed = 1.0;  // make this slower for outreaches
    // This declares the four drive chassis motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    DcMotor intakeMotor;

    DcMotor launcherMotor;

    DcMotor turretMotor;

    //this is declaring any other motors needed for robot
   // DcMotor armMotor;

//    DcMotor extensionMotor;

    Servo wrist, claw;

    ServoController controlHubServoController;

    double launcherPower = 0;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    float theta;

    /* this section is to create variables to use throughout program*/
//    int arm_down_position;
//    int Release_Position;
//    int arm_mid_position;
//    int arm_up_position;
//    int Arm_Pickup_Position;
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
        launcherMotor = hardwareMap.get(DcMotor.class,"launcher motor");
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");

        //this matches names of other motors in control hub to names created in beginning of this code
        controlHubServoController = hardwareMap.get(ServoController.class, "Control Hub");
        // extensionMotor=hardwareMap.get(DcMotor.class,"extensionMotor");
        //this matches names of servos like motors above
        // wrist = hardwareMap.get(Servo.class, "wrist");
        // claw = hardwareMap.get(Servo.class, "claw");

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

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    //this is the code that runs once you press play put game play code in this section
    public void loop() {
       //update this and reactivate them if you want message to display on driver station
        telemetry.addLine("Press cross (X) to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
//        telemetry.addLine("The left joystick sets the robot direction");
//        telemetry.addLine("Moving the right joystick left and right turns the robot");

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.cross) {
            imu.resetYaw();
        }

        //this is start of drive code

//         If you press the left bumper, you get a drive from the point of view of the robot
//         (much like driving an RC vehicle)
        if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

//        if (gamepad1.square) {
//            frontLeftDrive.setPower(1);
//        } else if (gamepad1.triangle) {
//            frontRightDrive.setPower(1);
//        } else if (gamepad1.cross) {
//            backLeftDrive.setPower(1);
//        } else if (gamepad1.circle) {
//            backRightDrive.setPower(1);
//        }

        telemetry.addData("Front Left drive power: ", frontLeftDrive.getPower());
        telemetry.addData("Front Right drive power: ", frontRightDrive.getPower());
        telemetry.addData("Back Left drive power: ", backLeftDrive.getPower());
        telemetry.addData("Back Right drive power: ", backRightDrive.getPower());


        if (gamepad2.left_bumper) {
            intakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(1);
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

        if (gamepad2.dpad_left) {
            turretMotor.setPower(1);
        } else if (gamepad2.dpad_right) {
            turretMotor.setPower(-1);
        } else {
            turretMotor.setPower(0);
        }


    //  if (gamepad2.right_trigger > 0) {
    //      launcherMotor.setPower(1);
    //  } else {
    //      launcherMotor.setPower(0);
    //  }



//        if (gamepad2.right_bumper == true && gamepad2.left_bumper == false) {
//            extensionMotor.setPower(1);
//            ((DcMotorEx) extensionMotor).setVelocity(1500);
//        } else if (gamepad2.right_bumper == false && gamepad2.left_bumper == false) {
//            extensionMotor.setPower(0);
//        }
//        if (gamepad2.left_bumper == true && gamepad2.right_bumper == false && extensionMotor.getCurrentPosition() > -50) {
//            extensionMotor.setPower(-0.75);
//        } else if (gamepad2.left_bumper == false && gamepad2.right_bumper == false) {
//            extensionMotor.setPower(0);
//        }
//
//        if (gamepad2.circle == true && gamepad2.square == false) {
//            claw.setPosition(1);
//        }
//
//        if (gamepad2.circle == false && gamepad2.square == true){
//            claw.setPosition(.45);
//        }
//
//        if (gamepad2.cross == true && gamepad2.triangle == false){
//            wrist.setPosition(0.5);
//        }
//
//        if (gamepad2.cross == false && gamepad2.triangle == true){
//            wrist.setPosition(1);
//        }

        }


    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

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
