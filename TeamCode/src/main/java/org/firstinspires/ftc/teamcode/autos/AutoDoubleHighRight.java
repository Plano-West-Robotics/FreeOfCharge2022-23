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
public class AutoDoubleHighRight extends LinearOpMode {
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
        api.pause(0.5);
        inchWorm.moveTo(21, 4);

        inchWorm.moveTo(21, 48, 0);
        inchWorm.moveTo(8, 48, 0);
        lift.setTargetPosition(SlidePresets.HIGH.position);
        api.pause(0.5);
        inchWorm.moveTo(8, 51, 0);
        api.pause(0.5);
        lift.setTargetPosition(SlidePresets.HIGH_SCORE.position);
        api.pause(0.5);
        claw.setPosition(1);
        inchWorm.moveTo(6.5, 48, 0);
        api.pause(0.5);

        lift.setTargetPosition(SlidePresets.STACK_5.position);
        inchWorm.moveTo(7, 50, 0);
        inchWorm.moveTo(7, 50, -Math.PI / 2);
        inchWorm.moveTo(new InchWorm2.Pose(-28, 49, -Math.PI / 2));
        claw.setPosition(0);
        api.pause(0.5);

        lift.setTargetPosition(SlidePresets.LOW.position);
        while (lift.isBusy()) {}
        inchWorm.moveTo(8, 49, 0);
        inchWorm.moveTo(8, 49, 0);
        lift.setTargetPosition(SlidePresets.HIGH.position);
        api.pause(0.5);
        inchWorm.moveTo(8, 54.5, 0);
        api.pause(0.5);
        lift.setTargetPosition(SlidePresets.HIGH_SCORE.position);
        api.pause(0.5);
        claw.setPosition(1);
        inchWorm.moveTo(8, 49, 0);
        api.pause(0.5);
        lift.setTargetPosition(SlidePresets.LOW.position);
        inchWorm.moveTo(-16.25, 51.25, 0);

        lift.setTargetPosition(SlidePresets.GROUND.position);

        inchWorm.moveTo(-16.25, 51, 0);

        // based on which tag was detected, move to the appropriate spot
        switch (detected_id) {
            case 1:
                inchWorm.moveTo(19.5, 51, 0);
                break;
            default:
            case 2:
                inchWorm.moveTo(-3.75, 51, 0);
                break;
            case 3:
                inchWorm.moveTo(-28, 51, 0);
                break;
        }
    }
}
