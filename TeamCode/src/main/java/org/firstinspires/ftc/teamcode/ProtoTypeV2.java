package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ProtoTypeV2 extends OpMode {
   private DcMotor motor1;
   private DcMotor motor2;

   @Override
   public void init() {


        motor1=hardwareMap.get(DcMotor.class, "motor1");
        motor2=hardwareMap.get(DcMotor.class, "motor2");


    }

    @Override
    public void loop() {

        if (gamepad1.square){
            motor1.setPower(1.0);
            motor2.setPower(1.0);
                    }

        else if (gamepad1.circle){
            motor1.setPower(-1.0);
            motor2.setPower(-1.0);
        }
        else{
            motor1.setPower(0.0);
            motor2.setPower(0.0);
        }
    }
}
