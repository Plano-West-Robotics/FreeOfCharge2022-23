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
public class AutoCameraLeft extends LinearOpMode {
    OpenCvWebcam camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    int detected_id = 0;

    @Override
    public void runOpMode() {
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

        // let the camera settle
        api.pause(5);

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if (currentDetections.size() != 0) {
            detected_id = currentDetections.get(0).id;
            telemetry.addData("Tag found", String.valueOf(detected_id));
            telemetry.update();
        }

        movementAPI.moveFor(0, 0.5, 0.2);
        movementAPI.stop();

        switch (detected_id) {
            case 1:
                movementAPI.moveFor(-90, 0.7, 1);
                movementAPI.moveFor(0, 0.5, 1.4);
                break;
            case 2:
                movementAPI.moveFor(0, 0.5, 1.4);
                break;
            case 3:
                movementAPI.moveFor(90, 0.7, 0.85);
                movementAPI.moveFor(0, 0.5, 1.4);
                break;
            default:
                movementAPI.moveFor(-90, 0.5, 1.75);
                movementAPI.moveFor(180, 0.5, 0.1);
                break;
        }

        movementAPI.stop();
    }
}
