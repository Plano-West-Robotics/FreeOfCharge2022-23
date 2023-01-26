package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class InchWorm2 {
    public static final double TICKS_PER_REV = 560;
    public static final double WHEEL_DIAMETER_INCHES = 3;
    // todo: tune these values
    public static final double WHEELBASE_WIDTH_INCHES = 0;
    public static final double WHEELBASE_LENGTH_INCHES = 0;
    public static final double WHEELBASE_DIAGONAL_INCHES = Math.hypot(WHEELBASE_WIDTH_INCHES, WHEELBASE_LENGTH_INCHES);
    public static final double TPI = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double DEGREES_PER_INCH = 360 / (WHEELBASE_DIAGONAL_INCHES * Math.PI);
    public static final double TICKS_PER_DEGREE = 1 / (DEGREES_PER_INCH / TPI);
    private final DcMotor fl;
    private final DcMotor fr;
    private final DcMotor bl;
    private final DcMotor br;

    private double speed = 0.5;

    private final LinearOpMode opMode;

    public InchWorm2(LinearOpMode mode) {
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
        pose = pose.toTicks();

        int[] targets = getWheelTargets(pose);

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
    }

    public void moveTo(double x, double y) {
        moveTo(new Pose(x, y));
    }
    public void moveTo(double x, double y, double theta) {
        moveTo(new Pose(x, y, theta));
    }

    private void setModes(DcMotor.RunMode mode) {
        fl.setMode(mode);
        fr.setMode(mode);
        bl.setMode(mode);
        br.setMode(mode);
    }

    private int[] getWheelTargets(Pose target) {
        int[] output = new int[4];

        // assuming pose has already been converted to ticks
        int rotation = (int) Math.floor((WHEELBASE_WIDTH_INCHES * TPI) * target.theta);

        // fl, fr, bl, br, respectively
        output[0] = (int) Math.floor(target.y - target.x + rotation);
        output[1] = (int) Math.floor(target.y + target.x - rotation);
        output[2] = (int) Math.floor(target.y + target.x + rotation);
        output[3] = (int) Math.floor(target.y - target.x - rotation);

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
        opMode.telemetry.addLine("start: " + start);
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
        double pow = Math.sqrt(Math.pow(getSpeedMultiplier(), 2) * Math.abs(1 - shift));

        opMode.telemetry.addLine("shift: " + shift);
        opMode.telemetry.addLine("pow: " + pow);

        // make sure power does not go over 0.5
        return Range.clip(pow, 0, getSpeedMultiplier());
    }

    public void setSpeedMultiplier(double newSpeed) {
        speed = newSpeed;
    }

    public double getSpeedMultiplier() {
        return speed;
    }

    public static class Pose {
        double x;
        double y;
        double theta = 0;

        public Pose(double X, double Y) {
            x = X;
            y = Y;
        }

        public Pose(double X, double Y, double angle) {
            x = X;
            y = Y;
            theta = angle;
        }

        public Pose toTicks() {
            return new Pose(this.x * TPI, this.y * TPI, this.theta / TICKS_PER_DEGREE);
        }
    }
}
