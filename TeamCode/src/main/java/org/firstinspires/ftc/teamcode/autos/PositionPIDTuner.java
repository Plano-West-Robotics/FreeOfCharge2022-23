package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PIDController;

@Autonomous(group="tune")
public class PositionPIDTuner extends LinearOpMode {
    /*
     * This class should be used to tune Ku with the Ziegler–Nichols method.
     * Requires a gamepad. Make sure to write down the tuned value, or it will be lost forever.
     */
    @Override
    public void runOpMode() {
        // tmp
        double Kp = 0;
        double Ki = 0;
        double Kd = 0;
        double scale = 0.15;
        DcMotor motor = hardwareMap.get(DcMotor.class, "slide");
        PIDController controller = new PIDController(Kp, Ki, Kd, 500);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        boolean lastLeftBumper = false;
        boolean lastRightBumper = false;
        boolean lastUp = false;
        boolean lastDown = false;
        boolean lastReset = false;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper && lastLeftBumper != gamepad1.left_bumper) {
                Kp -= scale;
                controller.setParams(Kp, Ki, Kd, 0);
                controller.reset();
            }

            if (gamepad1.right_bumper && lastRightBumper != gamepad1.right_bumper) {
                Kp += scale;
                controller.setParams(Kp, Ki, Kd, 0);
                controller.reset();
            }

            if (gamepad1.dpad_up && lastUp != gamepad1.dpad_up) {
                scale += 0.05;
            }
            if (gamepad1.dpad_down && lastDown != gamepad1.dpad_down) {
                scale -= 0.05;
            }

            if (gamepad1.x && lastReset != gamepad1.x) {
                controller.reset();
            }

            lastLeftBumper = gamepad1.left_bumper;
            lastRightBumper = gamepad1.right_bumper;

            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            lastReset = gamepad1.x;

            double error = motor.getCurrentPosition();
            double out = controller.calculate(error);

            telemetry.addData("target", 0);
            telemetry.addData("turn", out);
            telemetry.addData("error", error);
            telemetry.addData("Kd", Kd);
            telemetry.addData("scale", scale);
            telemetry.update();
            motor.setPower(out);
        }
    }
}
