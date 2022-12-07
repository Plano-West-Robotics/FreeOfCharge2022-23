package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
* Testing the linear slides and checking if they are operating 
* using DcMotor class. 
*/ 
@TeleOp(group="test")
public class SlideTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "slide");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            slideMotor.setPower(gamepad2.left_stick_y * 0.75);
            telemetry.addData("position", slideMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
