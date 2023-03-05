package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    private double lastFilterEstimate = 0;

    private double Kp;
    private double Ki;
    private double Kd;

    private final ElapsedTime timer = new ElapsedTime();
    private boolean started = false;

    public PIDController(double Kp, double Ki, double Kd, double target) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.target = target;
    }

    public double calculate(double current) {
        double error = target - current;

        // The PID loop has not been started yet, so ignore any time calculated before this point
        if (!started) {
            timer.reset();
            started = true;
            integralSum = 0;
            lastError = error;
        }


        double deltaTime = timer.seconds();

        double currentFilterEstimate = (0.6 * lastFilterEstimate) + (1-0.6) * (error - lastError);
        lastFilterEstimate = currentFilterEstimate;

        // derivative, AKA rate of change of the error
        double derivative = currentFilterEstimate / deltaTime;
        // calculate the Riemann sum of the error, also known as Forward Euler Integration.
        integralSum += error * deltaTime;

        // add all the parts together
        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        // track error over time
        lastError = error;
        // set timer back to 0 and resume counting
        timer.reset();

        return out;
    }

    public double calculateWithError(double error) {
        // The PID loop has not been started yet, so ignore any time calculated before this point
        if (!started) {
            timer.reset();
            started = true;
            integralSum = 0;
            lastError = error;
        }


        double deltaTime = timer.seconds();

        double currentFilterEstimate = (0.6 * lastFilterEstimate) + (1-0.6) * (error - lastError);
        lastFilterEstimate = currentFilterEstimate;

        // derivative, AKA rate of change of the error
        double derivative = currentFilterEstimate / deltaTime;
        // calculate the Riemann sum of the error, also known as Forward Euler Integration.
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
     * Resets the controller to state 0 (not started, no integralSum, no lastError).
     * Useful after changing controller parameters.
     * @see #setParams(double, double, double, double)
     */
    public void reset() {
        started = false;
    }

    /**
     * This method is for setting parameters to new values.
     * May be helpful to call {@link #reset()} after this.
     * @see #reset()
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

    /**
     * Sets the new target.
     * @param target New target position.
     */
    public void setTarget(double target) {
        this.target = target;
    }
}
