package org.firstinspires.ftc.teamcode.createdcode.enhancedautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous
public class AutoBlueColorSensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor sensor = hardwareMap.get(ColorSensor.class, "color");
        API api = new API(this);
        MovementAPI movementAPI = new MovementAPI(api);

        waitForStart();

        movementAPI.move(0, 0.7);
        api.pause(0.5);
        movementAPI.stop();

        api.pause(5);
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        int largest = api.getLargest(r, g, b);

        if (largest == g) {
            // move to position 1
            movementAPI.move(90, 0.7);
            api.pause(0.25);
        } else if (largest == r) {
            // move to position 2
            movementAPI.move(0, 0.7);
            api.pause(0.25);
        } else if (largest == b) {
            // move to position 3
            movementAPI.move(-90, 0.7);
            api.pause(0.25);
        } else {
            // Color sensor broke, move to backup position
            movementAPI.move(-180, 0.7);
            api.pause(0.5);
            movementAPI.move(90, 0.7);
            api.pause(0.7);
        }

        movementAPI.stop();
    }
}
