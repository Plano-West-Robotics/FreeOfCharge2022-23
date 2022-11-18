package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class PlaceConeColorSensorBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        ColorSensor sensor = hardwareMap.get(ColorSensor.class, "color");
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

        movementAPI.move(0, 0.5);
        api.pause(0.1);
        movementAPI.stop();

        motorLiftLeft.setPower(-1);
        motorLiftRight.setPower(-1);
        api.pause(0.25);
        motorLiftLeft.setPower(0);
        motorLiftRight.setPower(0);

        api.pause(2);

        claw.setPosition(1);
        api.pause(5);
        motorLiftLeft.setPower(1);
        motorLiftRight.setPower(1);
        api.pause(0.25);
        motorLiftLeft.setPower(0);
        motorLiftRight.setPower(0);

        movementAPI.move(180, 0.5);
        api.pause(0.5);
        movementAPI.move(-90, 0.5);
        api.pause(0.5);
        movementAPI.stop();

        movementAPI.move(0, 0.7);
        api.pause(0.4);
//        movementAPI.move(90, 0.7);
//        api.pause(0.15);
        movementAPI.stop();

        api.pause(5);
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        // move back to start position
//        movementAPI.move(-90, 0.7);
//        api.pause(0.05);
        movementAPI.move(-180, 0.7);
        api.pause(0.75);

        if (r < 100 && g < 100 && b < 100) {
            movementAPI.move(90, 0.7);
            api.pause(1);
            movementAPI.stop();

            terminateOpModeNow();
        }

        int largest = api.getLargest(r, g, b);

        api.print("r", String.valueOf(r));
        api.print("g", String.valueOf(g));
        api.print("b", String.valueOf(b));

        if (largest == g) {
            // move to position 1
            movementAPI.move(90, 0.7);
            api.pause(0.75);
            movementAPI.move(0, 0.7);
            api.pause(0.75);
        } else if (largest == r) {
            // move to position 2
            movementAPI.move(0, 0.7);
            api.pause(0.75);
        } else if (largest == b) {
            // move to position 3
            movementAPI.move(-90, 0.7);
            api.pause(0.75);
            movementAPI.move(0, 0.7);
            api.pause(0.75);
        }

        movementAPI.stop();
    }
}
