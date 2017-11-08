package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ShruthiJaganathan on 10/19/16.
 */

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="Intake Test", group="Test Opmode")
public class Test_Intake extends OpMode{
    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("leftmotor");
        rightMotor = hardwareMap.dcMotor.get("rightmotor");
    }

    @Override
    public void loop() {

        leftMotor.setPower(-1 * gamepad1.left_stick_y);
        rightMotor.setPower(-1 * gamepad1.right_stick_y);



    }
}
