package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * InchWorm: API for moving a certain number of inches. Make sure to tune DRIVE_TPI and STRAFE_TPI.
 */
public class InchWorm {
    /**
     * Ticks/inch for each motor (fl, fr, bl, br, respectively) going forward/backward.
     * Tune this using the DriveIPSTuner.
     */
    public static final int[] DRIVE_TPI = {57, 57, 57, 57};
    /**
     * Ticks/inch for each motor (fl, fr, bl, br, respectively) going left/right.
     * Tune this using the StrafeIPSTuner.
     */
    public static final int[] STRAFE_TPI = {-60, 62, 64, -60};

    /**
     * Whether to print debug values to telemetry. Defaults to false.
     */
    private static boolean debug = false;

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

        // reset encoders to 0
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // this is a temporary measure. modes will be reset once actually moving
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);

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
        int posFL = (int) (DRIVE_TPI[0] * inches);
        int posFR = (int) (DRIVE_TPI[1] * inches);
        int posBL = (int) (DRIVE_TPI[2] * inches);
        int posBR = (int) (DRIVE_TPI[3] * inches);

        // reset encoders to 0
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setTargetPosition(posFL);
        fr.setTargetPosition(posFR);
        bl.setTargetPosition(posBL);
        br.setTargetPosition(posBR);

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        while (isBusy()) {
            double flPower = ellipticCurve(fl.getCurrentPosition(), fl.getTargetPosition());
            double frPower = ellipticCurve(fr.getCurrentPosition(), fr.getTargetPosition());
            double blPower = ellipticCurve(bl.getCurrentPosition(), bl.getTargetPosition());
            double brPower = ellipticCurve(br.getCurrentPosition(), br.getTargetPosition());
            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);
        }

        stop();
    }

    /**
     * Strafe (left/right) a certain number of inches.
     * Negative inches means right.
     * @param inches Number of inches to move; positive means left, negative means right
     */
    public void strafe(double inches) {
        int posFL = (int) (STRAFE_TPI[0] * inches);
        int posFR = (int) (STRAFE_TPI[1] * inches);
        int posBL = (int) (STRAFE_TPI[2] * inches);
        int posBR = (int) (STRAFE_TPI[3] * inches);

        // reset encoders to 0
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setTargetPosition(posFL);
        fr.setTargetPosition(posFR);
        bl.setTargetPosition(posBL);
        br.setTargetPosition(posBR);

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        while (isBusy()) {
            double flPower = ellipticCurve(fl.getCurrentPosition(), fl.getTargetPosition());
            double frPower = ellipticCurve(fr.getCurrentPosition(), fr.getTargetPosition());
            double blPower = ellipticCurve(bl.getCurrentPosition(), bl.getTargetPosition());
            double brPower = ellipticCurve(br.getCurrentPosition(), br.getTargetPosition());
            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);
        }

        stop();
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
     * Set mode of all motors
     * @param mode Mode to set
     */
    private void setModes(DcMotor.RunMode mode) {
        fl.setMode(mode);
        fr.setMode(mode);
        bl.setMode(mode);
        br.setMode(mode);
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
        position = Math.abs(position);
        target = Math.abs(target);
        // midpoint of the radius, will shift the x-coordinates by `midpoint`
        double midpoint = target / 2;
        // 2.5% overshoot in the radius to make sure robot never fully stops
        double radius = midpoint * 1.025;

        // see the ellipse formula on wikipedia for more info
        double shift = Math.pow((position - midpoint) / radius, 2);
        double pow = Math.sqrt(Math.pow(0.5, 2) * Math.abs(1 - shift));

        opMode.telemetry.addData("position", position);
        opMode.telemetry.addData("target", target);
        opMode.telemetry.addData("shift", shift);
        opMode.telemetry.addData("pow", pow);
        if (debug) opMode.telemetry.update();

        // make sure power does not go over 0.5
        return Range.clip(pow, 0, 0.5);
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
