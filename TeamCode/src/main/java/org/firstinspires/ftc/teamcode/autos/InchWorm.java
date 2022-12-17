package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class InchWorm {
    // determine these values through measurement
    public static final int[] DRIVE_TPI = {0, 0, 0, 0};
    public static final int[] STRAFE_TPI = {0, 0, 0, 0};

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

    public boolean isBusy() {
        return fl.isBusy() ||
               fr.isBusy() ||
               bl.isBusy() ||
               br.isBusy() ||
               opMode.isStopRequested();
    }

    public void drive(double inches) {
        int posFL = fl.getCurrentPosition() + (int) (Math.round(DRIVE_TPI[0] * inches));
        int posFR = fr.getCurrentPosition() + (int) (Math.round(DRIVE_TPI[1] * inches));
        int posBL = bl.getCurrentPosition() + (int) (Math.round(DRIVE_TPI[2] * inches));
        int posBR = br.getCurrentPosition() + (int) (Math.round(DRIVE_TPI[3] * inches));

        fl.setTargetPosition(posFL);
        fr.setTargetPosition(posFR);
        bl.setTargetPosition(posBL);
        br.setTargetPosition(posBR);

        fl.setPower(0.5);
        fr.setPower(0.5);
        bl.setPower(0.5);
        br.setPower(0.5);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (isBusy()) {}

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void strafe(double inches) {
        int posFL = fl.getCurrentPosition() + (int) (Math.round(STRAFE_TPI[0] * inches));
        int posFR = fr.getCurrentPosition() + (int) (Math.round(STRAFE_TPI[1] * inches));
        int posBL = bl.getCurrentPosition() + (int) (Math.round(STRAFE_TPI[2] * inches));
        int posBR = br.getCurrentPosition() + (int) (Math.round(STRAFE_TPI[3] * inches));

        fl.setTargetPosition(posFL);
        fr.setTargetPosition(posFR);
        bl.setTargetPosition(posBL);
        br.setTargetPosition(posBR);

        fl.setPower(0.5);
        fr.setPower(0.5);
        bl.setPower(0.5);
        br.setPower(0.5);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (isBusy()) {}

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
