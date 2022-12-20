package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "test")
public class InchTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        InchWorm inchWorm = new InchWorm(this);

        waitForStart();

        inchWorm.drive(24);
        inchWorm.strafe(24);
        inchWorm.strafe(-24);
        inchWorm.drive(-24);
    }
}
