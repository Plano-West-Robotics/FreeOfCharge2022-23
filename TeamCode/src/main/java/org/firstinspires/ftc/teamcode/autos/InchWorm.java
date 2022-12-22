package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * InchWorm: API for moving a certain number of inches. Make sure to tune DRIVE_TPI and STRAFE_TPI.
 */
public class InchWorm {
    /**
     * Ticks/inch for each motor (fl, fr, bl, br, respectively) going forward/backward.
     * Tune this using the DriveIPSTuner.
     */
    public static final int[] DRIVE_TPI = {58, 55, 57, 56};
    /**
     * Ticks/inch for each motor (fl, fr, bl, br, respectively) going left/right.
     * Tune this using the StrafeIPSTuner.
     */
    public static final int[] STRAFE_TPI = {-60, 62, 64, -60};

    /**
     * Whether to print debug values to telemetry. Defaults to false.
     */
    private static boolean debug = true;

    private final DcMotor fl;
    private final DcMotor fr;
    private final DcMotor bl;
    private final DcMotor br;

    private final LinearOpMode opMode;

    public InchWorm(LinearOpMode mode) {
        opMode = mode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        fr = hardwareMap.get(DcMotor.class, "frontRight");
        bl = hardwareMap.get(DcMotor.class, "rearLeft");
        br = hardwareMap.get(DcMotor.class, "rearRight");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // this is a temporary measure. modes will be reset once actually moving
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
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
     * Drive (forward/backward) a certain number of inches. Negative inches means backwards.
     * @param inches Number of inches to move; positive means forward, negative means backwards
     */
    public void drive(double inches) {
        int posFL = fl.getCurrentPosition() + (int) (Math.round(DRIVE_TPI[0] * inches));
        int posFR = fr.getCurrentPosition() + (int) (Math.round(DRIVE_TPI[1] * inches));
        int posBL = bl.getCurrentPosition() + (int) (Math.round(DRIVE_TPI[2] * inches));
        int posBR = br.getCurrentPosition() + (int) (Math.round(DRIVE_TPI[3] * inches));

        fl.setTargetPosition(posFL);
        fr.setTargetPosition(posFR);
        bl.setTargetPosition(posBL);
        br.setTargetPosition(posBR);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double avgTarget = (Math.abs(posFL) + Math.abs(posFR) + Math.abs(posBL) + Math.abs(posBR)) / 4.0;

        while (isBusy()) {
            double avgPos = (Math.abs(fl.getCurrentPosition()) +
                    Math.abs(fr.getCurrentPosition()) +
                    Math.abs(bl.getCurrentPosition()) +
                    Math.abs(br.getCurrentPosition())) / 4.0;

            double power = ellipticCurve(avgPos, avgTarget, false);
            setPowers(power);
        }

        setPowers(0);
    }

    /**
     * Strafe (left/right) a certain number of inches.
     * Negative inches means right.
     * @param inches Number of inches to move; positive means left, negative means right
     */
    public void strafe(double inches) {
        int posFL = fl.getCurrentPosition() + (int) (Math.round(STRAFE_TPI[0] * inches));
        int posFR = fr.getCurrentPosition() + (int) (Math.round(STRAFE_TPI[1] * inches));
        int posBL = bl.getCurrentPosition() + (int) (Math.round(STRAFE_TPI[2] * inches));
        int posBR = br.getCurrentPosition() + (int) (Math.round(STRAFE_TPI[3] * inches));

        fl.setTargetPosition(posFL);
        fr.setTargetPosition(posFR);
        bl.setTargetPosition(posBL);
        br.setTargetPosition(posBR);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double avgTarget = (Math.abs(posFL) + Math.abs(posFR) + Math.abs(posBL) + Math.abs(posBR)) / 4.0;

        while (isBusy()) {
            double avgPos = (Math.abs(fl.getCurrentPosition()) +
                    Math.abs(fr.getCurrentPosition()) +
                    Math.abs(bl.getCurrentPosition()) +
                    Math.abs(br.getCurrentPosition())) / 4.0;

            double power = ellipticCurve(avgPos, avgTarget, false);
            setPowers(power);
        }

        setPowers(0);
    }

    /**
     * Set power of all motors
     * @param power Power to set
     */
    private void setPowers(double power) {
        opMode.telemetry.addData("power", power);
        if (debug) opMode.telemetry.update();
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    /**
     * Calculates average motor power based on an elliptical curve.
     * The ellipse is centered around (0, 0) with a vertical radius (b) of 1 or 0.5 (depending on whether we're strafing or not),
     * and a horizontal radius (a) of target.
     * See https://en.wikipedia.org/wiki/Ellipse for the ellipse equation.
     * @param current The current position
     * @param target  The target position of the whole movement
     * @param strafe Whether we're strafing or not. If false, sets the vertical radius (max power) to 0.5, otherwise 1.
     * @return An output power
     */
    private double ellipticCurve(double current, double target, boolean strafe) {
        opMode.telemetry.addData("current", current);
        opMode.telemetry.addData("target", target);
        current = Math.abs(current);
        target = Math.abs(target);
        /*
         * This peculiar piece of code is to prevent imaginary numbers when we sqrt later.
         * If direction is positive (driving forward/strafing left), current will be less than target,
         * so we can use an ellipse centered around the origin.
         * However, if the direction is negative (driving backward/strafing right),
         * current will be greater than target, so we use an ellipse that is shifted such that the left edge is at the target.
         * This makes the effect the same in that it slows down in an elliptical pattern.
         */
        double shift;
        if (current < target) {
            shift = Math.pow(current / (target + (target * 0.025)), 2);
        }
        else {
            shift = Math.pow((target - (target * 0.025)) / current, 2);
        }
        opMode.telemetry.addData("shift", shift);
        // if strafing, set multiplier to 1 instead of 0.5
        return Math.sqrt(Math.pow(strafe ? 1 : 0.5, 2) * (1 - shift));
    }

    /**
     * Enable/disable debug mode.
     * @param newDebug Whether or not to print debug data to telemetry
     */
    public void setDebug(boolean newDebug) {
        debug = newDebug;
    }

    /**
     * Return debug mode status
     * @return Whether or not debug mode is enabled
     */
    public boolean getDebug() {
        return debug;
    }
}
