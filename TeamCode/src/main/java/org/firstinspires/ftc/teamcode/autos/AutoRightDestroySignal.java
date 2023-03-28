package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LinearSlide;
import org.firstinspires.ftc.teamcode.SlidePresets;
import org.firstinspires.ftc.teamcode.vendor.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous
public class AutoRightDestroySignal extends LinearOpMode {
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
        LinearSlide lift = new LinearSlide(this);
        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.scaleRange(0, 0.65);
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

        // wait for the camera to detect something OR for 2.5 seconds to pass, whichever happens first
        double stopTime = getRuntime() + 0;
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
        lift.setTargetPosition(SlidePresets.LOW);
        lift.setPower(1);
        //api.pause(0.5);
        inchWorm.moveTo(-3, 0);
        inchWorm.moveTo(-3, 57);
        inchWorm.moveTo(-3, 51);

        //api.pause(1);
        lift.setTargetPosition(SlidePresets.HIGH);
        inchWorm.moveTo(6.5, 51.5, 0);
        //api.pause(0.2);
        inchWorm.moveTo(9, 53, 0);
        lift.setTargetPosition(SlidePresets.HIGH_SCORE);
        api.pause(0.2);
        claw.setPosition(1);
        //api.pause(0.5);

        lift.setTargetPosition(SlidePresets.STACK_5);
        //api.pause(0.5);
        inchWorm.setSpeedMultiplier(.75);
        inchWorm.moveTo(8, 49.5, -Math.PI / 2);
        //api.pause(0.5);
        inchWorm.moveTo(new InchWorm2.Pose(-24.5, 49.5, -Math.PI / 2));
        claw.setPosition(0);
        api.pause(0.25);

        lift.runToPosition(SlidePresets.LOW);
        inchWorm.setSpeedMultiplier(1);
        lift.setTargetPosition(SlidePresets.HIGH);
        inchWorm.moveTo(10.5, 49.5, 0);

        //api.pause(0.5);
        inchWorm.moveTo(10.5, 54.25, 0);
        api.pause(0.2);
        lift.setTargetPosition(SlidePresets.HIGH_SCORE);
        claw.setPosition(1);
        //api.pause(0.5);
        inchWorm.moveTo(10.5, 49.5, 0);

        lift.setTargetPosition(SlidePresets.STACK_4);
        //api.pause(0.5);
        inchWorm.setSpeedMultiplier(.75);
        inchWorm.moveTo(8, 49.5, -Math.PI / 2);
        //api.pause(0.5);
        inchWorm.moveTo(new InchWorm2.Pose(-24.5, 49.5, -Math.PI / 2));
        claw.setPosition(0);
        api.pause(0.25);

        lift.runToPosition(SlidePresets.LOW);
        inchWorm.setSpeedMultiplier(1);
        lift.setTargetPosition(SlidePresets.HIGH);
        inchWorm.moveTo(10.5, 49.5, 0);

        //api.pause(0.5);
        inchWorm.moveTo(10.5, 54.25, 0);
        api.pause(0.2);
        lift.setTargetPosition(SlidePresets.HIGH_SCORE);
        claw.setPosition(1);
        //api.pause(0.5);
        inchWorm.moveTo(10.5, 49.5, 0);

        //lift.setTargetPosition(SlidePresets.LOW);
        //inchWorm.moveTo(-16.25, 51.25, 0);

        lift.setTargetPosition(SlidePresets.GROUND);
        //inchWorm.moveTo(-16.25, 51, 0);

        // based on which tag was detected, move to the appropriate spot
        //inchWorm.setSpeedMultiplier(2);
        switch (detected_id) {
           case 1:
                inchWorm.moveTo(27.5, 48, 0);
                break;
            default:
            case 2:
                inchWorm.moveTo(-2, 48, 0);
                break;
            case 3:
                inchWorm.moveTo(-26, 48, 0);
                break;
        }
    }
}