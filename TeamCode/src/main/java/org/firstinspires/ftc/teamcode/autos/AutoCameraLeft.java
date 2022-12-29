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
public class AutoCameraLeft extends LinearOpMode {
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
        InchWorm inchWorm = new InchWorm(this);
        inchWorm.setDebug(true);
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

        // Allows the camera to settle
        api.pause(5);

        // get the first detection from the list
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if (currentDetections.size() != 0) {
            detected_id = currentDetections.get(0).id;
            telemetry.addData("Tag found", String.valueOf(detected_id));
            telemetry.update();
        }

        camera.stopStreaming();

        // move lift up to prevent cone from scraping on the floor
        lift.setTargetPosition(SlidePresets.LOW.position);
        lift.setPower(0.75);
        while (lift.isBusy()) {}

        // move forward to prevent scraping against the wall
        inchWorm.drive(4.5);
        // strafe to be inline with the junction
        inchWorm.strafe(-39.5);
        // drive until the cone is above the junction
        inchWorm.drive(22.5);
        // move lift up
        lift.setTargetPosition(SlidePresets.HIGH.position);
        while (lift.isBusy()) {}
        // drive towards junction
        inchWorm.drive(1);
        // move lift down & open claw
        lift.setTargetPosition(SlidePresets.GROUND.position);
        while (lift.isBusy()) {}
        lift.setPower(0);
        claw.setPosition(1);
        // drive back for next movements
        inchWorm.drive(-4.5);
        inchWorm.strafe(12.5);
        inchWorm.drive(24);

        // Based on which tag was detected, move to the corresponding position
        switch (detected_id) {
            case 1:
                inchWorm.strafe(47.5);
                break;
            case 2:
                inchWorm.strafe(22);
                break;
            default:
                break;
        }

        // move so that we are inside both squares
        inchWorm.drive(-13);
    }
}
