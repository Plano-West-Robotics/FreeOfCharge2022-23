package org.firstinspires.ftc.teamcode.aprilTag;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Locale;

@Autonomous(group = "Test")
public class AprilTagOpMode extends LinearOpMode {
    private AprilTagDetector detector;
    private int pos;

    @Override
    public void runOpMode() throws InterruptedException {
        detector = new AprilTagDetector(hardwareMap);
        while (!isStarted() && !isStopRequested()) {
            pos = detector.getPos();
            telemetry.addLine(String.format(Locale.ENGLISH, "\nDetected tag ID=%d", pos));
            telemetry.update();
            sleep(50);
        }
    }
}
