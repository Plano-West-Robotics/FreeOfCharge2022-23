package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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

        lift.setTargetPosition(1846);
        lift.setPower(0.75);
        while (lift.isBusy()) {}

        // move forward to prevent scraping against the wall
        inchWorm.drive(4.5);
        // strafe to be inline with the junction
        inchWorm.strafe(-14.75);
        // drive until the cone is above the junction
        inchWorm.drive(4.25);
        // move lift down
        lift.setTargetPosition(0);
        while (lift.isBusy()) {}
        // open claw & move lift back up
        claw.setPosition(1);
        lift.setTargetPosition(1846);
        while (lift.isBusy()) {}
        // move back to starting configuration
        inchWorm.drive(-4.25);
        lift.setTargetPosition(0);
        while (lift.isBusy()) {}
        lift.setPower(0);
        inchWorm.strafe(14.75);

        // Based on which tag was detected, move to the corresponding position
        switch (detected_id) {
            case 1:
                inchWorm.strafe(20.25);
                inchWorm.drive(35);
                break;
            case 2:
                inchWorm.strafe(-3.5);
                inchWorm.drive(35);
                break;
            case 3:
                inchWorm.strafe(-26.5);
                inchWorm.drive(35);
                break;
            default:
                // no tag was detected or camera broke, move to the fallback position
                inchWorm.drive(-35);
                inchWorm.drive(-4.5);
                break;
        }
    }
}
