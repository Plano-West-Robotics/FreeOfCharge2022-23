package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PIDController;

@Autonomous(group="tune")
public class TurnPIDTuner extends LinearOpMode {
    public static final double MAX_ANG_VEL = -188;
    /*
     * This class should be used to tune turn PID for InchWorm2.
     * Requires a gamepad. Make sure to write down the tuned values, or they will be lost forever.
     */
    @Override
    public void runOpMode() {
        double Kp = 5;
        double Ki = 0.15;
        double Kd = 0;
        double scale = 0.15;
        double target = 90;
        API api = new API(this);
        InchWorm2 inchWorm = new InchWorm2(this);
        PIDController controller = new PIDController(Kp, Ki, Kd, target);
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
                controller.setParams(Kp, Ki, Kd, target);
                controller.reset();
            }

            if (gamepad1.right_bumper && lastRightBumper != gamepad1.right_bumper) {
                Kd += scale;
                controller.setParams(Kp, Ki, Kd, target);
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

//            double current = inchWorm.getYaw(AngleUnit.DEGREES);
            double current = Math.toDegrees(inchWorm.tracker.currentPos.theta);
            double out = controller.calculate(current);
            out /= MAX_ANG_VEL;

            if (gamepad1.x) {
                out = 0;
                controller.reset();
            }

            telemetry.addData("target", target);
            telemetry.addData("out", out);
            telemetry.addData("error", String.format("%.2f", target - current));
            telemetry.addData("current", String.format("%.2f", current));
            telemetry.addData("Kp", String.format("%.2f", Kp));
            telemetry.addData("Ki", String.format("%.2f", Ki));
            telemetry.addData("Kd", String.format("%.2f", Kd));
            telemetry.addData("scale", scale);
            telemetry.update();

            inchWorm.moveWheels(0, 0, out, 0.5);
            inchWorm.tracker.update();
        }
    }
}
