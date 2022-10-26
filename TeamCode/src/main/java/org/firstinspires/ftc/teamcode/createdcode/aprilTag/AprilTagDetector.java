package org.firstinspires.ftc.teamcode.createdcode.aprilTag;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import org.openftc.apriltag.AprilTagDetection;


import java.util.ArrayList;

public class AprilTagDetector {
    private OpenCvWebcam webcam;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // private FtcDashboard dashboard = FtcDashboard.getInstance();
    // private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private HardwareMap hardwareMap;
    private int pos;

    public static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    private double fx = 578.272;
    private double fy = 578.272;
    private double cx = 402.145;
    private double cy = 221.506;

    // UNITS ARE METERS
    private double tagsize = 0.166;

    private int LEFT = 1;
    private int MIDDLE = 2;
    private int RIGHT = 3;

    private AprilTagDetection tagOfInterest = null;

    public AprilTagDetector(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        startCamera();
    }

    public void startCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagDetectionPipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public int getPos() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0) {
            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                {
                    tagOfInterest = tag;
                    pos = tagOfInterest.id;
                }
            }
        }

        return pos;
    }

    public void endStream() {
        webcam.stopStreaming();
    }
}
