package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class API {
    OpMode opMode;
    IMU imu;

    public API(OpMode opMode) {
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
        while (true) {
            if (!(opMode.getRuntime() < time)) break; // stop android studio from yelling
        }
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
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
