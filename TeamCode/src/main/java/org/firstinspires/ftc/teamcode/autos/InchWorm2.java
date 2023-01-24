package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class InchWorm2 {
    public static final double TICKS_PER_REV = 560;
    public static final double WHEEL_DIAMETER_INCHES = 3;
    public final double TPI;
    private final DcMotor fl;
    private final DcMotor fr;
    private final DcMotor bl;
    private final DcMotor br;

    private final LinearOpMode opMode;

    public InchWorm2(LinearOpMode mode) {
        double wheelCircumference = WHEEL_DIAMETER_INCHES * Math.PI;
        TPI = TICKS_PER_REV / wheelCircumference;

        opMode = mode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        fr = hardwareMap.get(DcMotor.class, "frontRight");
        bl = hardwareMap.get(DcMotor.class, "rearLeft");
        br = hardwareMap.get(DcMotor.class, "rearRight");

        // reset encoders to 0
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // this is a temporary measure. modes will be reset once actually moving
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        // counteract inertia
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // todo: add theta
    public void moveTo(Pose pose) {
        // convert pose in inches to pose in ticks
        pose.mul(TPI);
        // get current pose using forward kinematics
        Pose current = getCurrentPose();
        // get the distance to the target
        Pose diff = pose.getDiff(current);

        int[] targets = getWheelTargets(diff);

        int posFL = targets[0];
        int posFR = targets[1];
        int posBL = targets[2];
        int posBR = targets[3];

        int flStart = fl.getCurrentPosition();
        int frStart = fr.getCurrentPosition();
        int blStart = bl.getCurrentPosition();
        int brStart = br.getCurrentPosition();

        fl.setTargetPosition(posFL);
        fr.setTargetPosition(posFR);
        bl.setTargetPosition(posBL);
        br.setTargetPosition(posBR);

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        while (isBusy()) {
            opMode.telemetry.addLine("fl");
            double flPower = ellipticCurve(flStart, fl.getCurrentPosition(), fl.getTargetPosition());
            opMode.telemetry.addLine("fr");
            double frPower = ellipticCurve(frStart, fr.getCurrentPosition(), fr.getTargetPosition());
            opMode.telemetry.addLine("bl");
            double blPower = ellipticCurve(blStart, bl.getCurrentPosition(), bl.getTargetPosition());
            opMode.telemetry.addLine("br");
            double brPower = ellipticCurve(brStart, br.getCurrentPosition(), br.getTargetPosition());
            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);
            opMode.telemetry.update();
        }

        stop();
    }

    public void moveTo(double x, double y) {
        moveTo(new Pose(x, y));
    }

    private Pose getCurrentPose() {
        double currentX = (bl.getCurrentPosition() + fr.getCurrentPosition() - fl.getCurrentPosition() - br.getCurrentPosition()) / 4.0;
        double currentY = (fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4.0;

        return new Pose(currentX, currentY);
    }

    private void setModes(DcMotor.RunMode mode) {
        fl.setMode(mode);
        fr.setMode(mode);
        bl.setMode(mode);
        br.setMode(mode);
    }

    private int[] getWheelTargets(Pose target) {
        int[] output = new int[4];

        // fl, fr, bl, br, respectively
        output[0] = (int) Math.round(target.y - target.x);
        output[1] = (int) Math.round(target.y + target.x);
        output[2] = (int) Math.round(target.y + target.x);
        output[3] = (int) Math.round(target.y - target.x);

        return output;
    }

    /**
     * Whether motors are currently attempting to run to the target position.
     * Also includes a safeguard for stopping the opMode mid-move.
     * @return Whether all motors are busy, AND if the opMode is still running.
     */
    public boolean isBusy() {
        return fl.isBusy() &&
                fr.isBusy() &&
                bl.isBusy() &&
                br.isBusy() &&
                opMode.opModeIsActive();
    }

    /**
     * Stops all motors
     */
    private void stop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    /**
     * Calculates motor power based on an elliptic curve.
     * It creates an ellipse on a position vs power graph centered around (target / 2)
     * with a vertical radius of 0.5 and a horizontal radius of (midpoint * 1.025).
     * See https://en.wikipedia.org/wiki/Ellipse for the ellipse equation.
     * @param position The current position.
     * @param target The target position
     * @return Output power.
     */
    private double ellipticCurve(double start, double position, double target) {
        opMode.telemetry.addLine("position: " + position);
        opMode.telemetry.addLine("target: " + target);

        start = Math.abs(start);
        position = Math.abs(position);
        target = Math.abs(target);
        // midpoint of the radius, will shift the x-coordinates by `midpoint`
        double midpoint = (target - start) / 2;
        // 2.5% overshoot in the radius to make sure robot never fully stops
        double radius = midpoint * 1.025;

        // see the ellipse formula on wikipedia for more info
        double shift = Math.pow((position - midpoint) / radius, 2);
        double pow = Math.sqrt(Math.pow(0.5, 2) * Math.abs(1 - shift));

        opMode.telemetry.addLine("shift: " + shift);
        opMode.telemetry.addLine("pow: " + pow);

        // make sure power does not go over 0.5
        return Range.clip(pow, 0, 0.5);
    }

    public static class Pose {
        double x;
        double y;

        public Pose(double X, double Y) {
            x = X;
            y = Y;
        }

        public Pose getDiff(Pose other) {
            return new Pose(other.x - this.x, other.y - this.y);
        }

        public Pose mul(double n) {
            return new Pose(this.x * n, this.y * n);
        }
    }
}
