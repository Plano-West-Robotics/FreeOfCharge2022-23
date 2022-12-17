package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "tune")
public class DriveIPSTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        API api = new API(this);
        MovementAPI movementAPI = new MovementAPI(api);

        waitForStart();

        movementAPI.moveFor(0, 0.5, 1);

        while (opModeIsActive()) {
            telemetry.addLine(movementAPI.getFL().getCurrentPosition() + System.lineSeparator() +
                              movementAPI.getFR().getCurrentPosition() + System.lineSeparator() +
                              movementAPI.getBL().getCurrentPosition() + System.lineSeparator() +
                              movementAPI.getBR().getCurrentPosition() + System.lineSeparator()
            );
            telemetry.update();
        }
    }
}
