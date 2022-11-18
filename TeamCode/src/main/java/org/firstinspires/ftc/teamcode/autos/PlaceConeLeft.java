package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class PlaceConeLeft extends LinearOpMode {
    @Override
    public void runOpMode() {
        API api = new API(this);
        MovementAPI movementAPI = new MovementAPI(api);
        DcMotor motorLiftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        DcMotor motorLiftRight = hardwareMap.get(DcMotor.class, "liftRight");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(0);

        waitForStart();

        motorLiftLeft.setPower(1);
        motorLiftRight.setPower(1);
        api.pause(0.25);
        motorLiftLeft.setPower(0);
        motorLiftRight.setPower(0);
        api.pause(1);

        movementAPI.move(-90, 0.25);
        api.pause(1);
        movementAPI.stop();

        movementAPI.move(0, 0.5);
        api.pause(0.1);
        movementAPI.stop();
        api.pause(1);

        motorLiftLeft.setPower(-1);
        motorLiftRight.setPower(-1);
        api.pause(0.25);
        motorLiftLeft.setPower(0);
        motorLiftRight.setPower(0);

        api.pause(2);

        claw.setPosition(1);
        api.pause(5);
        movementAPI.move(180, 0.5);
        api.pause(0.5);
        motorLiftLeft.setPower(1);
        motorLiftRight.setPower(1);
        api.pause(0.25);
        motorLiftLeft.setPower(0);
        motorLiftRight.setPower(0);

        movementAPI.move(-90, 0.5);
        api.pause(0.75);
        movementAPI.stop();
    }
}
