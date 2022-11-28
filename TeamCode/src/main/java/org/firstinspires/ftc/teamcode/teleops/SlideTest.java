package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group="test")
public class SlideTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "slide");
        waitForStart();

        while (opModeIsActive()) {
            slideMotor.setPower(gamepad2.left_stick_y);
        }
    }
}
