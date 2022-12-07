package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vendor.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous
public class AutoCameraRight extends LinearOpMode {
    OpenCvWebcam camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    int detected_id = 0;

    @Override
    public void runOpMode() {
        // get the camera from the hardware map
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"));
        // set up the pipeline
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(
                CameraConstants.TAGSIZE,
                CameraConstants.FX,
                CameraConstants.FY,
                CameraConstants.CX,
                CameraConstants.CY);
        API api = new API(this);
        MovementAPI movementAPI = new MovementAPI(api);

        camera.setPipeline(aprilTagDetectionPipeline);
        // if the camera isn't detected in 2.5 seconds, stop attempting to connect
        camera.setMillisecondsPermissionTimeout(2500);
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

        // Allows the camera to settle
        api.pause(5);

        // get the first detection from the list
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if (currentDetections.size() != 0) {
            detected_id = currentDetections.get(0).id;
            telemetry.addData("Tag found", String.valueOf(detected_id));
            telemetry.update();
        }

        // move forward to prevent scraping against the wall
        movementAPI.moveFor(0, 0.5, 0.1);
        movementAPI.stop();

        // Based on which tag was detected, move to the corresponding position
        switch (detected_id) {
            case 1:
                movementAPI.moveFor(90, 0.7, 0.85);
                movementAPI.moveFor(0, 0.5, 1.4);
                break;
            case 2:
                movementAPI.moveFor(0, 0.5, 1.4);
                break;
            case 3:
                movementAPI.moveFor(-90, 0.7, 1.5);
                movementAPI.moveFor(0, 0.5, 1.4);
                break;
            default:
                // no tag was detected or camera broke, move to the fallback position
                movementAPI.moveFor(90, 0.5, 1.5);
                movementAPI.moveFor(180, 0.5, 0.1);
                break;
        }

        movementAPI.stop();
    }
}
