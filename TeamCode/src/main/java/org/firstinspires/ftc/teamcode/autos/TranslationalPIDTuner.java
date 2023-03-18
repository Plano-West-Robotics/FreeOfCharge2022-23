package org.firstinspires.ftc.teamcode.autos;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PIDController;

@Autonomous(group="tune")
public class TranslationalPIDTuner extends LinearOpMode {
    public static final double MAX_VEL = 2000;
    /*
     * This class should be used to tune translational PID for InchWorm2.
     * Requires a gamepad. Make sure to write down the tuned values, or they will be lost forever.
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        double Kp = 0;
        double Ki = 0;
        double Kd = 0;
        double scale = 0.15;
        API api = new API(this);
        InchWorm2 inchWorm = new InchWorm2(this);
        PIDController controller = new PIDController(Kp, Ki, Kd, 0);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        boolean lastLeftBumper = false;
        boolean lastRightBumper = false;
        boolean lastUp = false;
        boolean lastDown = false;

        api.waitForStart();

        InchWorm2.Pose target = new InchWorm2.Pose(0, 24, 0).toTicks();
        controller.setTarget(target.y);

        while (opModeIsActive()) {
            if (gamepad1.left_bumper && lastLeftBumper != gamepad1.left_bumper) {
                Kp -= scale;
                controller.setParams(Kp, Ki, Kd, target.y);
                controller.reset();
            }

            if (gamepad1.right_bumper && lastRightBumper != gamepad1.right_bumper) {
                Kp += scale;
                controller.setParams(Kp, Ki, Kd, target.y);
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

            InchWorm2.Pose current = inchWorm.tracker.currentPos;
            double out = controller.calculate(current.y);
            out /= MAX_VEL;

            if (gamepad1.x) {
                out = 0;
                controller.reset();
            }

            telemetry.addData("target", target.y);
            telemetry.addData("out", out);
            telemetry.addData("error", String.format("%.2f", target.y - current.y));
            telemetry.addData("Kp", String.format("%.2f", Kp));
            telemetry.addData("Ki", String.format("%.2f", Ki));
            telemetry.addData("Kd", String.format("%.2f", Kd));
            telemetry.addData("current", String.format("%.2f", current.y));
            telemetry.addData("scale", scale);
            telemetry.update();
            inchWorm.moveWheels(0, out, 0, 1);
            inchWorm.tracker.update();
        }
    }
}
