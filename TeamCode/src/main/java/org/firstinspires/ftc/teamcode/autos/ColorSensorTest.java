package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous
public class ColorSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        ColorSensor sensor = hardwareMap.get(ColorSensor.class, "color");
        API api = new API(this);

        waitForStart();
        while (opModeIsActive()) {
           api.print("r:", String.valueOf(sensor.red()));
           api.print("g:", String.valueOf(sensor.green()));
           api.print("b:", String.valueOf(sensor.blue()));
           telemetry.update();
        }
    }
}
