package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vendor.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AutoCameraRight extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    int detected_id = 0;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"));
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(
                CameraConstants.TAGSIZE,
                CameraConstants.FX,
                CameraConstants.FY,
                CameraConstants.CX,
                CameraConstants.CY);
        API api = new API(this);
        MovementAPI movementAPI = new MovementAPI(api);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        // let the camera settle
        api.pause(5);

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if (currentDetections.size() != 0) {
            detected_id = currentDetections.get(0).id;
            telemetry.addData("Tag found", String.valueOf(detected_id));
            telemetry.update();
        }

        movementAPI.move(0, 0.5);
        api.pause(0.1);
        movementAPI.stop();

        switch (detected_id) {
            case 1:
                movementAPI.move(90, 0.7);
                api.pause(0.85);
                movementAPI.move(0, 0.5);
                api.pause(1.4);
                break;
            case 2:
                movementAPI.move(0, 0.5);
                api.pause(1.4);
                break;
            case 3:
                movementAPI.move(-90, 0.7);
                api.pause(1.5);
                movementAPI.move(0, 0.5);
                api.pause(1.4);
                break;
            default:
                movementAPI.move(90, 0.5);
                api.pause(1.5);
                movementAPI.move(180, 0.5);
                break;
        }

        movementAPI.stop();
    }
}
