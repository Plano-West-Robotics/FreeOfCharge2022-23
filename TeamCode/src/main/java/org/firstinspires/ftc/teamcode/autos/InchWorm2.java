package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

/**
 * InchWorm2 is a movement system aimed at making autonomous coding easy and effective.
 */
public class InchWorm2 {
    /**
     * Encoder ticks per motor revolution for your drive motors. You can find this information online.
     */
    public static final double TICKS_PER_REV = 560;
    /**
     * Diameter of your mecanum wheels in inches.
     */
    public static final double WHEEL_DIAMETER_INCHES = 3;
    /**
     * Encoder ticks per inch rotated
     */
    public static final double TPI = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    private final DcMotor fl;
    private final DcMotor fr;
    private final DcMotor bl;
    private final DcMotor br;
    public final IMU imu;
    public final PositionTracker tracker = new PositionTracker();
    private int loopsCorrect = 0;
    /**
     * Maximum translational velocity in encoder ticks/second. Find this using the SpeedTuner
     */
    private static final double MAX_VEL = 2000;
    /**
     * Maximum angular velocity in degrees/second. Find this using the SpeedTuner.
     */
    private static final double MAX_ANG_VEL = -188;
    /**
     * PID controllers. <b>coefficients for controllerX and controllerY should be THE SAME!</b>
     * Tune controllerX and controllerY with the TranslationalPIDTuner, and tune controllerTheta with the TurnPIDTuner.
     */
    private final PIDController controllerX = new PIDController(10, 0.05, 0, 0);
    private final PIDController controllerY = new PIDController(10, 0.05, 0, 0);
    private final PIDController controllerTheta = new PIDController(10, 0.15, 0, 0);

    private final LinearOpMode opMode;

    /**
     * speed multiplier, multiplied by the PID outputs.
     */
    private double speed = 1;

    /**
     * Whether to draw a representation of the robot to FTC Dashboard or not.
     */
    private static final boolean DASHBOARD_DRAW = true;
    private final FtcDashboard dashboard;
    // TODO: change these values
    private static final Pose startPos = new Pose(0, 0, 0);

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

        dashboard = FtcDashboard.getInstance();
    }

    /**
     * Move to a specific position on the field.
     * @param pose Position to move to. For now, must be in inches and radians.
     */
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

            double voltageCompensation = 12 / getBatteryVoltage();
            moveWheels(out.x, out.y, out.theta, getSpeedMultiplier() * voltageCompensation);
            tracker.update();
        }

        stop();
    }

    /**
     * Move to a certain (x, y) on the field.
     * @param x x coordinate to move to. for now, must be in inches
     * @param y y coordinate to move to. for now, must be in inches
     */
    public void moveTo(double x, double y) {
        moveTo(new Pose(x, y));
    }

    /**
     * Move to a certain (x, y, θ) on the field.
     * @param x x coordinate to move to. for now, must be in inches.
     * @param y y coordinate to move to. for now, must be in inches.
     * @param theta angle to turn to. for now, must be in radians.
     */
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

    /**
     * Get yaw from the IMU.
     * @param angleUnit Unit of the result.
     * @return yaw angle from the IMU, in `angleUnit` units.
     */
    public double getYaw(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    /**
     * Get yaw from the IMU in radians.
     * @return yaw angle from the IMU in radians.
     */
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

    /**
     * Set the speed multiplier of the robot.
     * @param x new speed to set
     */
    public void setSpeedMultiplier(double x) {
        speed = x;
    }

    /**
     * Get the current speed multiplier.
     * @return current speed multiplier
     */
    public double getSpeedMultiplier() {
        return speed;
    }

    private void drawPose(Pose currentPose) {
        if (!DASHBOARD_DRAW) return;

        TelemetryPacket packet = new TelemetryPacket();
        Pose p = startPos.add(currentPose.rot(startPos.theta));
        double l = 0.5; // customize this if needed

        packet.fieldOverlay()
                .strokeCircle(p.x, p.y, 1)
                .strokeLine(Math.cos(p.theta) + p.x, Math.sin(p.theta) + p.y, l * Math.cos(p.theta) + p.x, l * Math.sin(p.theta) + p.y);
        dashboard.sendTelemetryPacket(packet);
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

        /**
         * Convert a pose where x and y are in inches to a pose where x and y in ticks.
         * @return a pose where x and y in ticks.
         */
        public Pose toTicks() {
            return new Pose(this.x * TPI, this.y * TPI, this.theta);
        }

        /**
         * Normalizes theta into [0, 2π).
         * @see super.modAngle(double)
         * @return A new pose with theta normalized into [0, 2π).
         */
        public Pose normalizeAngle() {
            return new Pose(this.x, this.y, modAngle(this.theta));
        }

        /**
         * Rotates this pose by an angle.
         * @param angle angle to rotate by
         * @return new pose that is rotated by the angle
         */
        public Pose rot(double angle) {
            double rotX = this.x * Math.cos(angle) - this.y * Math.sin(angle);
            double rotY = this.x * Math.sin(angle) + this.y * Math.cos(angle);

            return new Pose(rotX, rotY, this.theta);
        }

        public Pose add(Pose other) {
            return new Pose(this.x + other.x, this.y + other.y, this.theta + other.theta);
        }

        @NonNull
        public String toString() {
            return "x: " + x + System.lineSeparator() + "y: " + y + System.lineSeparator() + "theta: " + theta;
        }

        /**
         * Converts theta from radians to degrees.
         * @return a pose with radians converted to degrees
         */
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

        private int flOffset = 0;
        private int frOffset = 0;
        private int blOffset = 0;
        private int brOffset = 0;
        private double yawOffset = 0;

        /**
         * Set this to the diameter of the robot, in inches
         */
        private final int TRACKWIDTH = 0;

        /**
         * Override the current pose estimate.
         * @param pose Pose to relocalize to. x and y must be in inches, and theta must be in radians.
         */
        public void setPoseEstimate(Pose pose) {
            pose = pose.toTicks().normalizeAngle();
            double turn = (pose.theta * TRACKWIDTH) * TPI;

            int currentFL = fl.getCurrentPosition();
            int currentFR = fr.getCurrentPosition();
            int currentBL = bl.getCurrentPosition();
            int currentBR = br.getCurrentPosition();

            int fl = (int) (pose.y - pose.x + turn);
            int fr = (int) (pose.y + pose.x - turn);
            int bl = (int) (pose.y + pose.x + turn);
            int br = (int) (pose.y - pose.x - turn);


            lastFL = fl;
            lastFR = fr;
            lastBL = bl;
            lastBR = br;

            flOffset = fl - currentFL;
            frOffset = fr - currentFR;
            blOffset = bl - currentBL;
            brOffset = br - currentBR;
            yawOffset = angleDiff(pose.theta, getYaw());

            currentPos = pose;
        }

        private double sinc(double x) {
            return x == 0 ? 1 : Math.sin(x) / x;
        }

        // this function doesn't really have a standard name, but it's similar to sinc so cosc it is
        // not to be confused with cosec
        private double cosc(double x) {
            return x == 0 ? 0 : (1 - Math.cos(x)) / x;
        }

        /**
         * Updates the current position estimate. The more you call this, the better.
         */
        public void update() {
            int newFL = fl.getCurrentPosition() + flOffset;
            int newFR = fr.getCurrentPosition() + frOffset;
            int newBL = bl.getCurrentPosition() + blOffset;
            int newBR = br.getCurrentPosition() + brOffset;

            double newYaw = getYaw() + yawOffset;
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

            drawPose(currentPos);
        }
    }
}
