package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Reset IMU", group = "util")
public class IMUReset extends LinearOpMode {
    @Override
    public void runOpMode() {
        API api = new API(this);

        api.waitForStart();

        api.reset();
    }
}
