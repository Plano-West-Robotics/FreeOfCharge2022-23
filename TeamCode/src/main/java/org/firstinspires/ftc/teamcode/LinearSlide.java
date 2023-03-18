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
        setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Set the target position of the lift.
     * @param pos position to go to.
     */
    public void setTargetPosition(int pos) {
        motor.setTargetPosition(pos);
    }

    /**
     * Set the target position of the lift.
     * @param pos position to go to.
     */
    public void setTargetPosition(SlidePresets pos) {
        setTargetPosition(pos.position); // lmao
    }

    /**
     * Checks if the motor is busy
     * @return Whether the motor is running towards a certain position AND the opMode is still active.
     */
    public boolean isBusy() {
        return motor.isBusy() && opMode.opModeIsActive();
    }

    /**
     * Continually run towards a position.
     * @param pos position to run to.
     */
    public void runToPosition(int pos) {
        setTargetPosition(pos);
        while (isBusy()) {}
    }

    /**
     * Continually run towards a position.
     * @param pos position to run to.
     */
    public void runToPosition(SlidePresets pos) {
        runToPosition(pos.position);
    }

    /**
     * Set the power of the motor.
     * @param power New power
     */
    public void setPower(double power) {
        motor.setPower(power);
    }
}
