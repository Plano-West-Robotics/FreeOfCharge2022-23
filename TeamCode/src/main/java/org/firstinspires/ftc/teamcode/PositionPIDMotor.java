package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Motor wrapper to run to position using PID. Tune this using the PositionPIDTuner.
 * Requires an encoder to be plugged in with the motor.
 */
public class PositionPIDMotor {
    private final DcMotor motor;
    private final PIDController controller;

    private double Kp;
    private double Ki;
    private double Kd;

    private int targetPosition = 0;

    private double speedMultiplier = 0.75;

    /**
     * Initialize using default PID parameters.
     *
     * @param motor Motor to manage.
     */
    public PositionPIDMotor(DcMotor motor) {
        this.motor = motor;

        // TODO: tune these values
        Kp = 0;
        Ki = 0;
        Kd = 0;

        controller = new PIDController(Kp, Ki, Kd, targetPosition);
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Initialize using custom PID parameters.
     *
     * @param motor Motor to manage.
     * @param Kp    Proportional gain
     * @param Ki    Integral gain
     * @param Kd    Derivative gain
     */
    public PositionPIDMotor(DcMotor motor, double Kp, double Ki, double Kd) {
        this.motor = motor;

        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        controller = new PIDController(Kp, Ki, Kd, targetPosition);
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Electronically reads the current position of the motor.
     *
     * @return The current position of the motor
     */
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    /**
     * The position that the motor was last commanded to move to.
     *
     * @return The target position of the motor.
     */
    public int getTargetPosition() {
        return targetPosition;
    }

    /**
     * Set the target position of the motor.
     *
     * @param newTarget The new target position
     */
    public void setTargetPosition(int newTarget) {
        targetPosition = newTarget;
        controller.setParams(Kp, Ki, Kd, targetPosition);
        controller.reset();
    }

    /**
     * Change the PID parameters.
     * Really only useful during tuning, but someone smarter than me may be able to find a use for this
     *
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     */
    public void setParams(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        controller.setParams(Kp, Ki, Kd, targetPosition);
        controller.reset();
    }

    /**
     * Whether the motor is currently attempting to move to the target position.
     *
     * @return whether the motor is currently attempting to move to the target position.
     */
    public boolean isBusy() {
        // TODO: change this threshold based on testing
        return Math.abs(getCurrentPosition() - targetPosition) < 5;
    }

    /**
     * Calculates the PID value and attempts to move towards the target position. Useful during loops.
     */
    public void updatePosition() {
        double pidval = controller.calculate(getCurrentPosition());

        motor.setPower(-pidval * getSpeedMultiplier());
    }

    /**
     * Calculates the PID value and continuously attempts to move towards the target position.
     * <b>WARNING: THIS METHOD BLOCKS</b>
     */
    public void run() {
        while (isBusy()) {
            updatePosition();
        }
    }

    public void setSpeedMultiplier(double speed) {
        speedMultiplier = speed;
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }
}
