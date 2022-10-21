package org.firstinspires.ftc.teamcode.createdcode.oldthings;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous
public class getServoPosition extends LinearOpMode {


    @Override
    public void runOpMode() {


        waitForStart();


        telemetry.update();
        sleep(10000);
    }
}
