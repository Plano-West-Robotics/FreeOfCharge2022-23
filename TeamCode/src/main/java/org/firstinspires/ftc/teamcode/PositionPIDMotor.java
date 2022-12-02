package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class PositionPIDMotor {
    private final DcMotor motor;
    private final PIDController controller;

    private double Kp;
    private double Ki;
    private double Kd;

    private int targetPosition = 0;

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

    public PositionPIDMotor(DcMotor motor, double Kp, double Ki, double Kd) {
        this.motor = motor;

        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        controller = new PIDController(Kp, Ki, Kd, targetPosition);
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(int newTarget) {
        targetPosition = newTarget;
        controller.setParams(Kp, Ki, Kd, targetPosition);
        controller.reset();
    }

    public void setParams(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        controller.setParams(Kp, Ki, Kd, targetPosition);
        controller.reset();
    }

    public boolean isBusy() {
        // TODO: change this threshold based on testing
        return Math.abs(getCurrentPosition() - targetPosition) < 5;
    }

    public void updatePosition() {
        if (isBusy()) {
            double pidval = controller.calculate(getCurrentPosition());

            motor.setPower(pidval);
        } else {
            motor.setPower(0);
        }
    }

    public void run() {
        while (isBusy()) {
            updatePosition();
        }
    }
}
