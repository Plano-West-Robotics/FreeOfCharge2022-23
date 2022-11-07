package org.firstinspires.ftc.teamcode.createdcode.enhancedautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous
public class ColorSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        ColorSensor sensor = hardwareMap.get(ColorSensor.class, "color");
        waitForStart();
        while (opModeIsActive()) {
           telemetry.addData("r:", sensor.red());
           telemetry.addData("g:", sensor.green());
           telemetry.addData("b:", sensor.blue());
           telemetry.update();
        }
    }
}
