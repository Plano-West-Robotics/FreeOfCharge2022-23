package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Linear slide wrapper which makes it a bit easier to work with.
 */
public class LinearSlide {
    private final DcMotorEx motor;
    private final LinearOpMode opMode;

    public LinearSlide(LinearOpMode opMode) {
        this.opMode = opMode;
        motor = opMode.hardwareMap.get(DcMotorEx.class, "slide");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTargetPosition(int pos) {
        motor.setTargetPosition(pos);
    }

    public void setTargetPosition(SlidePresets pos) {
        setTargetPosition(pos.position); // lmao
    }

    public boolean isBusy() {
        return motor.isBusy() && opMode.opModeIsActive();
    }

    public void runToPosition(int pos) {
        setTargetPosition(pos);
        while (isBusy()) {}
    }

    public void runToPosition(SlidePresets pos) {
        runToPosition(pos.position);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }
}
