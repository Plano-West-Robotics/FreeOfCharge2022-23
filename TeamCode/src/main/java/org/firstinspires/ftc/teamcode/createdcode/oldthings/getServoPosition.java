package org.firstinspires.ftc.teamcode.createdcode.oldthings;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous
public class getServoPosition extends LinearOpMode {
    private Servo armBaseServo;
    private DcMotor armTurnMotor;
    @Override
    public void runOpMode() {
        armBaseServo = hardwareMap.get(Servo.class, "armBaseServo");

        waitForStart();

        armBaseServo.setPosition(0.5);


        telemetry.addData("Position is", armBaseServo.getPosition());
        telemetry.update();
        sleep(10000);
    }
}
