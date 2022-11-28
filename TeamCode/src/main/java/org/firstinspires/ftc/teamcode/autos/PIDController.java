package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Class which manages a PID controller.
 * <br>
 * Kp, Ki, and Kd should be tuned and passed into the constructor.
 * <br>
 * For more information, see https://www.ctrlaltftc.com/the-pid-controller
 */
public class PIDController {
    private double target;
    private double integralSum = 0;
    private double lastError;

    private double Kp;
    private double Ki;
    private double Kd;

    private ElapsedTime timer = new ElapsedTime();
    private boolean started = false;

    public PIDController(double Kp, double Ki, double Kd, double target) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.target = target;
    }

    public double calculate(double error) {
        // The PID loop has not been started yet, so ignore any time calculated before this point
        if (!started) {
            timer.reset();
            started = true;
        }

        double deltaTime = timer.seconds();

        // derivative, AKA rate of change of the error
        double derivative = (error - lastError) / deltaTime;
        // we do a little riemann sum
        integralSum += error * deltaTime;

        // add all the parts together
        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        // track error over time
        lastError = error;
        // set timer back to 0 and resume counting
        timer.reset();

        return out;
    }

    /**
     * This method is for recalibrating tuning and should not be used during normal operation.
     * @param Kp     Proportional gain
     * @param Ki     Integral gain
     * @param Kd     Derivative gain
     * @param target Target point to reach
     */
    public void setParams(double Kp, double Ki, double Kd, double target) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.target = target;
    }
}
