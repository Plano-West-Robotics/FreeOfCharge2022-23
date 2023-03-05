package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PIDController;

import java.util.List;

public class InchWorm2 {
    public static final double TICKS_PER_REV = 560;
    public static final double WHEEL_DIAMETER_INCHES = 3;
    public static final double TPI = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    private final DcMotor fl;
    private final DcMotor fr;
    private final DcMotor bl;
    private final DcMotor br;
    public final IMU imu;
    public final PositionTracker tracker = new PositionTracker();
    private int loopsCorrect = 0;
    // todo: tune these values
    private static final double MAX_VEL = 2000;
    private static final double MAX_ANG_VEL = -188;
    private final PIDController controllerX = new PIDController(10, 0.05, 0, 0);
    private final PIDController controllerY = new PIDController(10, 0.05, 0, 0);
    private final PIDController controllerTheta = new PIDController(5, 0.15, 0, 0);

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

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
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
    private double ellipticCurve(double position, double target) {
        opMode.telemetry.addLine("position: " + position);
        opMode.telemetry.addLine("target: " + target);

        position = Math.abs(position);
        target = Math.abs(target);
        // midpoint of the radius, will shift the x-coordinates by `midpoint`
        double midpoint = target / 2;
        // 2.5% overshoot in the radius to make sure robot never fully stops
        double radius = midpoint * 1.025;

        // see the ellipse formula on wikipedia for more info
        double shift = Math.pow((position - midpoint) / radius, 2);
        double pow = Math.sqrt(Math.abs(1 - shift));

        opMode.telemetry.addLine("shift: " + shift);
        opMode.telemetry.addLine("pow: " + pow);

        // make sure power does not go over 0.5
        return Range.clip(pow, 0, 1);
    }

    public void moveTo(Pose pose) {
        // convert pose in inches to pose in ticks & normalize angle to [-π, π] radians
        pose = pose.toTicks().normalizeAngle();
        controllerX.setTarget(pose.x);
        controllerY.setTarget(pose.y);
        controllerTheta.setTarget(Math.toDegrees(pose.theta));
        controllerX.reset();
        controllerY.reset();
        controllerTheta.reset();
        Pose current = tracker.currentPos.normalizeAngle();

        while (isBusy(pose, current)) {
            current = tracker.currentPos.normalizeAngle();
            opMode.telemetry.addLine(current.toDegrees().toString());
            double angError = Math.toDegrees(angleDiff(pose.theta, current.theta));
            opMode.telemetry.addData("angError", angError);
            Pose out = new Pose(controllerX.calculate(current.x), controllerY.calculate(current.y), controllerTheta.calculateWithError(angError));

            out = out.rot(current.theta);
            out = new Pose(out.x / MAX_VEL, out.y / MAX_VEL, out.theta / MAX_ANG_VEL);
            opMode.telemetry.addLine(out.toString());
            opMode.telemetry.update();

            moveWheels(out.x, out.y, out.theta, 12 / getBatteryVoltage());
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

    public void moveX(double n) {
        Pose current = tracker.currentPos;
        Pose target = new Pose(current.x + n, current.y, current.theta);
        moveTo(target);
    }

    public void moveY(double n) {
        Pose current = tracker.currentPos;
        Pose target = new Pose(current.x, current.y + n, current.theta);
        moveTo(target);
    }

    public void turn(double n) {
        Pose current = tracker.currentPos;
        Pose target = new Pose(current.x, current.y, current.theta + n);
        moveTo(target);
    }

    public void turnTo(double n) {
        Pose current = tracker.currentPos;
        Pose target = new Pose(current.x, current.y, n);
        moveTo(target);
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
    public boolean isBusy(Pose target, Pose current) {
        if (
                Math.abs(target.x - current.x) <= 20 &&
                Math.abs(target.y - current.y) <= 20 &&
                Math.abs(Math.toDegrees(angleDiff(target.theta, current.theta))) <= 5
        ) {
            loopsCorrect++;
        } else loopsCorrect = 0;

        return loopsCorrect <= 35 && opMode.opModeIsActive();
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
     * normalizes theta into [0, 2π)
     * @param theta angle to normalize in radians
     * @return theta normalized into [0, 2π)
     */
    private static double modAngle(double theta) {
        // convert to degrees because mod 2pi doesn't work?
        double angle = Math.toDegrees(theta);

        angle += 360;
        angle %= 360;

        // convert back to radians when done
        return Math.toRadians(angle);
    }

    /**
     * Returns the real (smallest) difference between two angles.
     * @param a first angle (in radians)
     * @param b second angle (in radians)
     * @return smallest difference between the two angles, within range [-π, π)
     */
    private static double angleDiff(double a, double b) {
        double diff = a - b;
        if (diff >= Math.PI) diff -= 2 * Math.PI;
        if (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }

    public double getYaw(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    public double getYaw() {
        return getYaw(AngleUnit.RADIANS);
    }

    private double getBatteryVoltage() {
        // returns the root of mean of the squares of all the battery voltages
        double totalSquares = 0;
        int numSeen = 0;
        for (VoltageSensor sensor : opMode.hardwareMap.voltageSensor) {
            totalSquares += Math.pow(sensor.getVoltage(), 2);
            numSeen++;
        }

        return Math.sqrt(totalSquares / numSeen);
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
            return new Pose(this.x, this.y, modAngle(this.theta));
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

        public String toString() {
            return "x: " + x + System.lineSeparator() + "y: " + y + System.lineSeparator() + "theta: " + theta;
        }

        public Pose toDegrees() {
            return new Pose(this.x, this.y, Math.toDegrees(this.theta));
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
            double yawDiff = angleDiff(newYaw, currentPos.theta);

            int flDiff = newFL - lastFL;
            int frDiff = newFR - lastFR;
            int blDiff = newBL - lastBL;
            int brDiff = newBR - lastBR;

            double yDiff = ((flDiff + frDiff + blDiff + brDiff) / 4.0);
            double xDiff = ((blDiff + frDiff - flDiff - brDiff) / 4.0);

            double expX = cosc(yawDiff);
            double expY = sinc(yawDiff);

            Pose posDiff = new Pose(yDiff * expX + xDiff * expY, yDiff * expY - xDiff * expX, yawDiff);
            posDiff = posDiff.rot(-currentPos.theta);

            currentPos = currentPos.add(posDiff);

            lastFL = newFL;
            lastFR = newFR;
            lastBL = newBL;
            lastBR = newBR;
        }
    }
}
