package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(group = "tune")
public class TurnTPRTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        API api = new API(this);
        MovementAPI movementAPI = new MovementAPI(api);

        // reset encoders to 0
        movementAPI.getFL().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        movementAPI.getFR().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        movementAPI.getBL().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        movementAPI.getBR().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        movementAPI.getFL().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        movementAPI.getFR().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        movementAPI.getBL().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        movementAPI.getBR().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        movementAPI.move(0, -1, 0.5);
        api.pause(1);

        while (opModeIsActive()) {
            telemetry.addLine(api.getHeading(AngleUnit.RADIANS) + System.lineSeparator() +
                    movementAPI.getFL().getCurrentPosition() + System.lineSeparator() +
                    movementAPI.getFR().getCurrentPosition() + System.lineSeparator() +
                    movementAPI.getBL().getCurrentPosition() + System.lineSeparator() +
                    movementAPI.getBR().getCurrentPosition() + System.lineSeparator()
            );
        }
    }
}
