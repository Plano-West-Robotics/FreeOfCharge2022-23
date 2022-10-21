package org.firstinspires.ftc.teamcode.createdcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class encoderTester extends OpMode {


    @Override
    public void loop() {


        telemetry.addData("right_stick_y", gamepad1.right_stick_y);


    }

    @Override
    public void init() {
        /*





         */
    }

}
