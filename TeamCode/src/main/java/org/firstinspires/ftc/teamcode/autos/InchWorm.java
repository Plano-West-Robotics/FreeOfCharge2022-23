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

        int avgTarget = Math.round((Math.abs(posFL) + Math.abs(posFR) + Math.abs(posBL) + Math.abs(posBR)) / 4.0f);

        while (isBusy()) {
            int avgPos = Math.round((Math.abs(fl.getCurrentPosition()) +
                    Math.abs(fr.getCurrentPosition()) +
                    Math.abs(bl.getCurrentPosition()) +
                    Math.abs(br.getCurrentPosition())) / 4.0f);

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

        int avgTarget = Math.round((Math.abs(posFL) + Math.abs(posFR) + Math.abs(posBL) + Math.abs(posBR)) / 4.0f);

        while (isBusy()) {
            int avgPos = Math.round((Math.abs(fl.getCurrentPosition()) +
                    Math.abs(fr.getCurrentPosition()) +
                    Math.abs(bl.getCurrentPosition()) +
                    Math.abs(br.getCurrentPosition())) / 4.0f);

            double power = ellipticCurve(avgPos, avgTarget, true);
            setPowers(power);
        }

        setPowers(0);
    }

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
     * The ellipse centered around (0, 0) with a vertical radius (b) of 1 or 0.5 (depending on whether we're strafing or not),
     * and a horizontal radius (a) of target.
     * @param current The current position
     * @param target  The target position of the whole movement
     * @param strafe Whether we're strafing or not. If false, sets the vertical radius (max power) to 0.5, otherwise 1.
     * @return An output power
     */
    private double ellipticCurve(int current, int target, boolean strafe) {
        opMode.telemetry.addData("current", current);
        opMode.telemetry.addData("target", target);
        double shift;
        if (current < target) shift = Math.pow(Math.abs((double) current) / (double) target, 2);
        else shift = Math.pow(Math.abs((double) target) / (double) current, 2);
        opMode.telemetry.addData("shift", shift);
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
