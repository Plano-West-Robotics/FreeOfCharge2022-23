package org.firstinspires.ftc.teamcode.createdcode.enhancedautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoBlueSubstationParkingBehavior extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fl = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor fr = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor rl = hardwareMap.get(DcMotor.class, "rearLeft");
        DcMotor rr = hardwareMap.get(DcMotor.class, "rearRight");

        API api = new API(this);
        MovementAPI movementAPI = new MovementAPI(api, fl, fr, rl, rr);

        waitForStart();

        movementAPI.move(90, 0.7);
        api.pause(0.7);
        movementAPI.stop();
    }
}
