package org.firstinspires.ftc.teamcode.kinematics;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

@Config
public class Poser {
    Hardware hardware;
    Telemetry telemetry;
    EncoderIntegrator integrator;

    double speed;

    public static double POS_STOP_THRESHOLD = 5;
    public static double YAW_STOP_THRESHOLD = 0.5;
    public static int LIFT_STOP_THRESHOLD = 7;
    public static int SLOWDOWN_THRESHOLD_FACTOR = 40;

    public Poser(Hardware hardware, double speed, FieldVector initialPos, double initialYaw) {
        this.hardware = hardware;
        this.speed = speed;
        this.telemetry = hardware.opMode.telemetry;
        this.integrator = new EncoderIntegrator(hardware, initialPos, initialYaw, telemetry);
    }

    private Pose getPose() {
        return new Pose(
                integrator.pos,
                integrator.imuYaw,
                hardware.getLiftEncoder()
        );
    }

    public static double ellipticalCurve(double x, double slowdownThreshold) {
        if (Math.abs(x) > slowdownThreshold) {
            return Math.signum(x);
        } else {
            return Math.signum(x) * Math.sqrt(1 - Math.pow((Math.abs(x) / slowdownThreshold) - 1, 2));
        }
    }

    public void goTo(NullablePose targetPose) {
        Pose currentPose = this.getPose();

        this.goToPose(new Pose(
                targetPose.pos != null ? targetPose.pos : currentPose.pos,
                targetPose.yaw != null ? targetPose.yaw : currentPose.yaw,
                targetPose.liftPos != null ? targetPose.liftPos : currentPose.liftPos
        ));
    }

    public void goToPose(Pose targetPose) {
        int loopsCorrect = 0;

        while (true) {
            this.integrator.update();

            Pose currentPose = this.getPose();

            FieldVector posDiff = targetPose.pos.sub(currentPose.pos);
            double yawDiff = targetPose.yaw - currentPose.yaw;
            int liftDiff = targetPose.liftPos - currentPose.liftPos;

            if (
                    Math.abs(posDiff.magnitude().valInMM()) < POS_STOP_THRESHOLD
                            && Math.abs(yawDiff) < YAW_STOP_THRESHOLD
                            && Math.abs(liftDiff) < LIFT_STOP_THRESHOLD
            ) {
                loopsCorrect += 1;
            } else {
                loopsCorrect = 0;
            }

            if (loopsCorrect >= 120) {
                hardware.stop();
                hardware.lift.setPower(0);
                break;
            }

            telemetry.addData("posX", currentPose.pos.x);
            telemetry.addData("posY", currentPose.pos.y);

            telemetry.addData("posDiff", posDiff);
            telemetry.addData("posXDiff", posDiff.x);
            telemetry.addData("posYDiff", posDiff.y);
            telemetry.addData("yawDiff", yawDiff);
            telemetry.addData("liftDiff", liftDiff);

            FieldVector posPower = posDiff.normalized().mul(ellipticalCurve(posDiff.magnitude().valInMM(), POS_STOP_THRESHOLD * SLOWDOWN_THRESHOLD_FACTOR));
            double yawPower = ellipticalCurve(yawDiff, YAW_STOP_THRESHOLD * SLOWDOWN_THRESHOLD_FACTOR);
            double liftPower = ellipticalCurve(liftDiff, LIFT_STOP_THRESHOLD * SLOWDOWN_THRESHOLD_FACTOR);

            telemetry.addData("posPower", posPower);
            telemetry.addData("posXPower", posPower.x);
            telemetry.addData("posYPower", posPower.y);
            telemetry.addData("yawPower", yawPower);
            telemetry.addData("liftPower", liftPower);
            telemetry.addData("loopsCorrect", loopsCorrect);
            hardware.dashboardTelemetry.drawTarget(targetPose, currentPose);
            hardware.dashboardTelemetry.drawRobot(currentPose.pos, currentPose.yaw);
            telemetry.update();

            posPower = posPower.rot(-currentPose.yaw);
            hardware.move(
                    posPower.x.valInDefaultUnits(),
                    posPower.y.valInDefaultUnits(),
                    yawPower,
                    speed
            );
            hardware.lift.setPower(liftPower);
        }
    }
}
