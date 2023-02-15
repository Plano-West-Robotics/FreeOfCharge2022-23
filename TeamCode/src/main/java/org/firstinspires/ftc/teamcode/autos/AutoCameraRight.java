package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SlidePresets;
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
        // set up the API
        API api = new API(this);
        // set up InchWorm
        InchWorm2 inchWorm = new InchWorm2(this);
        // disabled for now because slide is broken
        DcMotor lift = hardwareMap.get(DcMotor.class, "slide");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.scaleRange(0, 0.5);
        claw.setPosition(0);

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

        api.waitForStart();

        // wait for the camera to detect something OR for 5 seconds to pass, whichever happens first
        double stopTime = getRuntime() + 5;
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        while (currentDetections.size() == 0 && getRuntime() < stopTime) {
            currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        }

        // get the first detection from the list
        // we check size again in case the while loop was stopped due to time
        if (currentDetections.size() != 0) {
            detected_id = currentDetections.get(0).id;
            telemetry.addData("Tag found", String.valueOf(detected_id));
            telemetry.update();
        }

        try {
            camera.stopStreaming();
        } catch (Exception ignored) {}

        // move lift up to prevent cone from scraping on the floor
        lift.setTargetPosition(SlidePresets.LOW.position);
        lift.setPower(0.75);

        inchWorm.moveTo(21, 4);
        lift.setTargetPosition(SlidePresets.HIGH.position);
        inchWorm.moveTo(21, 39.75, Math.PI / 2);
        inchWorm.moveTo(24.75, 39.75, Math.PI / 2);
        // move lift down & open claw
        lift.setTargetPosition(SlidePresets.MEDIUM.position);
        while (lift.isBusy()) {}
        claw.setPosition(1);
        lift.setTargetPosition(SlidePresets.GROUND.position);
        inchWorm.moveTo(21, 39.75, Math.PI / 2);
        inchWorm.moveTo(21, 51.25, 0);

        // Based on which tag was detected, move to the corresponding position
        switch (detected_id) {
            case 2:
                inchWorm.moveTo(-3.75, 51.25, 0);
                break;
            case 3:
                inchWorm.moveTo(-22, 51.25, 0);
                break;
        }
    }
}
