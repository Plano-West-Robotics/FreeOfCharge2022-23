package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class API {
    LinearOpMode opMode;
    IMU imu;

    public API(LinearOpMode opMode) {
        this.opMode = opMode;
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                // TODO: change these parameters if they are not accurate
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
        /*
            TODO: this shouldn't really be needed i think, but it doesn't hurt.
            TODO: remove it pending communication with the FTC discord people
         */
        imu.resetYaw();
    }

    public void pause(double seconds) {
        double time = opMode.getRuntime() + seconds;
        while (opMode.getRuntime() < time && !opMode.isStopRequested()) {}
    }

    public int getLargest(int x, int y, int z) {
        return Math.max(z, Math.max(x, y));
    }

    public void print(String s) {
        opMode.telemetry.addLine(s);
    }

    public void print(String caption, String value) {
        opMode.telemetry.addData(caption, value);
    }

    /**
     * Re-interprets current heading (AKA yaw) to be 0.
     * Make sure to call this function after turning.
     * @see API#getHeading()
     */
    public void reset() {
        imu.resetYaw();
    }

    /**
     * Get the current rotation around the Z-Axis (known as heading or yaw) since the last time reset() was called.
     * This value will be normalized to be within [-180, 180) <b>degrees, not radians</b>. It follows the right-hand-rule:
     * Positive values are <b>counter-clockwise</b> around the axis, negative values are <b>clockwise</b>.
     * @see API#reset()
     * @return the rotation since the last time reset() was called
     */
    public double getHeading() {
        return getHeading(AngleUnit.DEGREES);
    }

    /**
     * Get the current rotation around the Z-Axis (known as heading or yaw) since the last time reset() was called.
     * You can provide an AngleUnit to get the result in either degrees or radians
     * This value will be normalized to be within [-180, 180) degrees (or [-π, π) radians). It follows the right-hand-rule:
     * Positive values are <b>counter-clockwise</b> around the axis, negative values are <b>clockwise</b>.
     * @see API#reset()
     * @param angleUnit The unit for the result to be in
     * @return the rotation since the last time reset() was called, in `angleUnit`s.
     */
    public double getHeading(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    /**
     * LinearOpMode's waitForStart, but doesn't crash the robot when the stop button is pressed.
     */
    public void waitForStart() {
        while (!opMode.isStarted() && !opMode.isStopRequested());
    }
}
