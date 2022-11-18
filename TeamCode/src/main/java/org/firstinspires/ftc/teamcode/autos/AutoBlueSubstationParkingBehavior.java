package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoBlueSubstationParkingBehavior extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        API api = new API(this);
        MovementAPI movementAPI = new MovementAPI(api);

        waitForStart();

        movementAPI.move(90, 0.7);
        api.pause(0.7);
        movementAPI.stop();
    }
}
