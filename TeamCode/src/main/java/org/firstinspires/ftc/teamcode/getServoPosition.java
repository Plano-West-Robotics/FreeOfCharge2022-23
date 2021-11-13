package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous
public class getServoPosition extends LinearOpMode {
    private Servo bucketServo;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        telemetry.addData("Position is", bucketServo.getPosition());
        telemetry.update();
    }
}
