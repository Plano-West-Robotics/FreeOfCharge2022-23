package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PIDController;

public class InchWorm2 {
    public static final double TICKS_PER_REV = 560;
    public static final double WHEEL_DIAMETER_INCHES = 3;
    public static final double TPI = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    private final DcMotor fl;
    private final DcMotor fr;
    private final DcMotor bl;
    private final DcMotor br;
    private final IMU imu;
    public final PositionTracker tracker = new PositionTracker();
    // todo: tune these values
    private final PIDController controllerX = new PIDController(10, 0.05, 0, 0);
    private final PIDController controllerY = new PIDController(10, 0.05, 0, 0);
    private final PIDController controllerTheta = new PIDController(0, 0, 0, 0);

    private double speed = 0.5;

    private final LinearOpMode opMode;

    public InchWorm2(LinearOpMode mode) {
        opMode = mode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        fr = hardwareMap.get(DcMotor.class, "frontRight");
        bl = hardwareMap.get(DcMotor.class, "rearLeft");
        br = hardwareMap.get(DcMotor.class, "rearRight");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                // TODO: change these parameters if they are not accurate
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
        imu.resetYaw();

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

    public void moveTo(Pose pose) {
        // convert pose in inches to pose in ticks & normalize angle to [-π, π] radians
        pose = pose.toTicks().normalizeAngle();
        controllerX.setTarget(pose.x);
        controllerY.setTarget(pose.y);
        controllerTheta.setTarget(pose.theta);
        controllerX.reset();
        controllerY.reset();
        controllerTheta.reset();

        while (isBusy()) {
            Pose current = tracker.currentPos;
            Pose out = new Pose(controllerX.calculate(current.x), controllerY.calculate(current.y), controllerTheta.calculate(current.theta));

            out = out.rot(current.theta);

            moveWheels(out.x, out.y, out.theta, getSpeedMultiplier());
            tracker.update();
        }

        stop();
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

    /**
     * Whether motors are currently attempting to run to the target position.
     * Also includes a safeguard for stopping the opMode mid-move.
     * @return Whether all motors are busy, AND if the opMode is still running.
     */
    public boolean isBusy() {
        // todo: make this real
        return opMode.opModeIsActive();
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

    public void moveWheels(double powerX, double powerY, double turn, double speed) {
        double flPower = (powerY - powerX + turn) * speed;
        double frPower = (powerY + powerX - turn) * speed;
        double blPower = (powerY + powerX + turn) * speed;
        double brPower = (powerY - powerX - turn) * speed;

        double scale = Math.max(1, (Math.abs(powerY) + Math.abs(turn) + Math.abs(powerX)) * Math.abs(speed)); // shortcut for max(abs([fl,fr,bl,br]))
        flPower /= scale;
        frPower /= scale;
        blPower /= scale;
        brPower /= scale;

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);

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

    public double getYaw(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    public double getYaw() {
        return getYaw(AngleUnit.RADIANS);
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
            return new Pose(this.x * TPI, this.y * TPI, this.theta);
        }

        public Pose normalizeAngle() {
            double radians = this.theta;
            while (radians > Math.PI) radians -= 2 * Math.PI;
            while (radians < Math.PI) radians += 2 * Math.PI;

            return new Pose(this.x, this.y, radians);
        }

        // only rotates movement vectors, does not touch theta
        public Pose rot(double angle) {
            double rotX = this.x * Math.cos(angle) - this.y * Math.sin(angle);
            double rotY = this.x * Math.sin(angle) + this.y * Math.cos(angle);

            return new Pose(rotX, rotY, this.theta);
        }

        public Pose add(Pose other) {
            return new Pose(this.x + other.x, this.y + other.y, this.theta + other.theta);
        }
    }

    public class PositionTracker {
        public Pose currentPos = new Pose(0, 0, 0);
        private int lastFL = 0;
        private int lastFR = 0;
        private int lastBL = 0;
        private int lastBR = 0;

        private double sinc(double x) {
            return x == 0 ? 1 : Math.sin(x) / x;
        }

        // this function doesn't really have a standard name, but it's similar to sinc so cosc it is
        // not to be confused with cosec
        private double cosc(double x) {
            return x == 0 ? 0 : (1 - Math.cos(x)) / x;
        }

        public void update() {
            int newFL = fl.getCurrentPosition();
            int newFR = fr.getCurrentPosition();
            int newBL = bl.getCurrentPosition();
            int newBR = br.getCurrentPosition();

            double newYaw = getYaw();
            double yawDiff = newYaw - currentPos.theta;

            int flDiff = newFL - lastFL;
            int frDiff = newFR - lastFR;
            int blDiff = newBL - lastBL;
            int brDiff = newBR - lastBR;

            double yDiff = ((flDiff + frDiff + blDiff + brDiff) / 4.0);
            double xDiff = ((blDiff + frDiff - flDiff - brDiff) / 4.0);

            double expX = cosc(yawDiff);
            double expY = sinc(yawDiff);

            Pose posDiff = new Pose(yDiff * expX + xDiff * expY, yDiff * expY - xDiff * expX, yawDiff);
            posDiff = posDiff.rot(currentPos.theta);

            currentPos = currentPos.add(posDiff);

            lastFL = newFL;
            lastFR = newFR;
            lastBL = newBL;
            lastBR = newBR;
        }
    }
}
