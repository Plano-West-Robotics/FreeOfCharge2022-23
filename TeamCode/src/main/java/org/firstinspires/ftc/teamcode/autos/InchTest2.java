package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "test")
public class InchTest2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        InchWorm2 inchWorm = new InchWorm2(this);
        API api = new API(this);

        api.waitForStart();

        inchWorm.moveTo(24, 0);
        inchWorm.moveTo(24, 24);
        inchWorm.moveTo(24, 0);
        inchWorm.moveTo(0, 0);

    }
}
