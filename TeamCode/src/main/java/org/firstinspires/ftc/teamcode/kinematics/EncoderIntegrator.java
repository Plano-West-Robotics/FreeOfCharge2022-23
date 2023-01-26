package org.firstinspires.ftc.teamcode.kinematics;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

public class EncoderIntegrator {
    Hardware hardware;

    FieldVector pos;
    // `yaw` is the robot angle. `imuYaw` is the last measured value from the imu. usually just
    // offset by a constant but that could be different in the future, i.e. radians or smth.
    double yaw;

    int fl;
    int fr;
    int bl;
    int br;
    double imuYaw;

    Telemetry telemetry;

    public final double MM_PER_ENCODER_TICK = (96 * Math.PI) / 537.7;

    public EncoderIntegrator(Hardware hardware, FieldVector initialPos, double initialYaw, Telemetry telemetry) {
        this.hardware = hardware;
        this.pos = initialPos;
        this.yaw = initialYaw;
        this.telemetry = telemetry;

        this.fl = hardware.fl.getCurrentPosition();
        this.fr = hardware.fr.getCurrentPosition();
        this.bl = hardware.bl.getCurrentPosition();
        this.br = hardware.br.getCurrentPosition();
        this.imuYaw = hardware.getImuYaw();
    }

    private static double sinc(double x) {
        if (x == 0) {
            return 1;
        } else {
            return Math.sin(x) / x;
        }
    }

    // this function has no generally well-known name, but its similar to `sinc` so im calling it `cosc`
    private static double cosc(double x) {
        if (x == 0) {
            return 0;
        } else {
            return (1 - Math.cos(x)) / x;
        }
    }

    public void update() {
        int newFl = hardware.fl.getCurrentPosition();
        int newFr = hardware.fr.getCurrentPosition();
        int newBl = hardware.bl.getCurrentPosition();
        int newBr = hardware.br.getCurrentPosition();
        double newImuYaw = hardware.getImuYaw();

        double flDiff = newFl - this.fl;
        double frDiff = newFr - this.fr;
        double blDiff = newBl - this.bl;
        double brDiff = newBr - this.br;
        double imuYawDiff = newImuYaw - this.imuYaw;

        double relativeYDiff = ((flDiff + frDiff + blDiff + brDiff) / 4.) * MM_PER_ENCODER_TICK;
        double relativeXDiff = ((flDiff - frDiff - blDiff + brDiff) / 4.) * MM_PER_ENCODER_TICK;

        double imuYawDiffRadians = Math.toRadians(imuYawDiff);
        double poseExponentiationX = cosc(imuYawDiffRadians);
        double poseExponentiationY = sinc(imuYawDiffRadians);

        FieldVector relativePosDiff = FieldVector.inMM(
                relativeYDiff * poseExponentiationX + relativeXDiff * poseExponentiationY,
                relativeYDiff * poseExponentiationY - relativeXDiff * poseExponentiationX
        );

        FieldVector posDiff = relativePosDiff.rot(this.imuYaw);

        this.pos = this.pos.add(posDiff);
        this.yaw += imuYawDiff;

        this.fl = newFl;
        this.fr = newFr;
        this.bl = newBl;
        this.br = newBr;
        this.imuYaw = newImuYaw;
        this.imuYaw %= 360;
    }
}
