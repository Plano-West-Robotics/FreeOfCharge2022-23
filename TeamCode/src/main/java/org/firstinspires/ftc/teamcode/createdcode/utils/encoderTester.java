package org.firstinspires.ftc.teamcode.createdcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class encoderTester extends OpMode {
    DcMotor arm1, arm2;

    @Override
    public void loop() {
        arm1.setPower(gamepad1.right_stick_y);
        arm2.setPower(gamepad1.right_stick_y);
        telemetry.addData("right_stick_y", gamepad1.right_stick_y);
        telemetry.addData("arm1 encoder value", arm1.getCurrentPosition());
        telemetry.addData("arm2 encoder value", arm2.getCurrentPosition());
    }

    @Override
    public void init() {
        arm1 = hardwareMap.get(DcMotor.class, "armOne");
        arm2 = hardwareMap.get(DcMotor.class, "armTwo");
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

}
