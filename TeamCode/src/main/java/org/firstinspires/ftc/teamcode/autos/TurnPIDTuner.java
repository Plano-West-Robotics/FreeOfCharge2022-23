package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PIDController;

@Autonomous(group="tune")
public class TurnPIDTuner extends LinearOpMode {
    /*
     * This class should be used to tune Ku with the Ziegler–Nichols method.
     * Requires a gamepad. Make sure to write down the tuned value, or it will be lost forever.
     */
    @Override
    public void runOpMode() {
        // tmp
        double Kp = 0.15;
        double Ki = 0;
        double Kd = 0;
        double scale = 0.15;
        API api = new API(this);
        MovementAPI movementAPI = new MovementAPI(api);
        PIDController controller = new PIDController(Kp, Ki, Kd, 0);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        boolean lastLeftBumper = false;
        boolean lastRightBumper = false;
        boolean lastUp = false;
        boolean lastDown = false;

        api.waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper && lastLeftBumper != gamepad1.left_bumper) {
                Kd -= scale;
                controller.setParams(Kp, Ki, Kd, 0);
                controller.reset();
            }

            if (gamepad1.right_bumper && lastRightBumper != gamepad1.right_bumper) {
                Kd += scale;
                controller.setParams(Kp, Ki, Kd, 0);
                controller.reset();
            }

            if (gamepad1.dpad_up && lastUp != gamepad1.dpad_up) {
                scale += 0.05;
            }
            if (gamepad1.dpad_down && lastDown != gamepad1.dpad_down) {
                scale -= 0.05;
            }

            lastLeftBumper = gamepad1.left_bumper;
            lastRightBumper = gamepad1.right_bumper;

            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            double error = api.getHeading();
            double out = controller.calculate(error);

            if (gamepad1.x) {
                out = 0;
                controller.reset();
            }

            telemetry.addData("target", 0);
            telemetry.addData("turn", out);
            telemetry.addData("error", error);
            telemetry.addData("Kd", Kd);
            telemetry.addData("scale", scale);
            telemetry.update();
            movementAPI.move(0, 0, out, 0.5, false);
        }
    }
}
