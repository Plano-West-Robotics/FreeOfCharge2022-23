package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group="tune")
public class TurnPIDTuner extends LinearOpMode {
    /*
     * This class should be used to tune Ku with the Ziegler–Nichols method.
     * Requires a gamepad. Make sure to write down the tuned value, or it will be lost forever.
     */
    @Override
    public void runOpMode() {
        double Kp = 0;
        double Ki = 0;
        double Kd = 0;
        API api = new API(this);
        MovementAPI movementAPI = new MovementAPI(api);
        PIDController controller = new PIDController(Kp, Ki, Kd, 0);

        boolean lastLeftBumper = false;
        boolean lastRightBumper = false;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper && lastLeftBumper != gamepad1.left_bumper) {
                Kp -= 0.15;
                controller.setParams(Kp, Ki, Kd, 0);
            }

            if (gamepad1.right_bumper && lastRightBumper != gamepad1.right_bumper) {
                Kp += 0.15;
                controller.setParams(Kp, Ki, Kd, 0);
            }

            lastLeftBumper = gamepad1.left_bumper;
            lastRightBumper = gamepad1.right_bumper;

            double error = api.getHeading();
            double out = controller.calculate(error);

            telemetry.addLine(
                    "Current Kp: " + Kp + System.lineSeparator() +
                    "Current error: " + error + System.lineSeparator() +
                    "Calculated PID value: " + out + System.lineSeparator()
            );
            movementAPI.move(0, 0, out, 1, false);
        }
    }
}
