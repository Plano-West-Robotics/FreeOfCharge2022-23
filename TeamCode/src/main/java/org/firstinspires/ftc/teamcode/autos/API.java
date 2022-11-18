package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class API {
    OpMode opMode;

    public API(OpMode opMode) {
        this.opMode = opMode;
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
}
