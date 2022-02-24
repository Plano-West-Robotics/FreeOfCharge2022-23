package org.firstinspires.ftc.teamcode.createdcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp
public class ObjectDetectionTest extends LinearOpMode
{
    public static int minH = 100;
    public static int minS = 100;
    public static int minV = 100;
    public static int maxH = 120;
    public static int maxS = 255;
    public static int maxV = 255;

    OpenCvWebcam webcam;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));


        webcam.setPipeline(new SamplePipeline());


        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();


        waitForStart();

        while (opModeIsActive())
        {
            dashboard.startCameraStream(webcam, 0);

            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();


            if(gamepad1.a)
            {

                webcam.stopStreaming();

            }

            sleep(100);
        }
    }


    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input)
        {
            Mat blurredImage = new Mat();
            Mat hsvImage = new Mat();
            Mat mask = new Mat();
            Mat morphOutput = new Mat();

            // remove some noise
            Imgproc.blur(input, blurredImage, new Size(7, 7));

            // convert the frame to HSV
            Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);


            Scalar minValues = new Scalar(minH, minS, minV);
            Scalar maxValues = new Scalar(maxH, maxS, maxV);
            Core.inRange(hsvImage, minValues, maxValues, mask);

            //erodes the little things

            Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
            Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));

            Imgproc.erode(mask, morphOutput, erodeElement);

            Imgproc.dilate(mask, morphOutput, dilateElement);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            Mat output = input.clone();

            List<Rect> rectList= new ArrayList<>();

            // find contours
            Imgproc.findContours(morphOutput, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
            if (hierarchy.size().height > 0 && hierarchy.size().width > 0)
            {
                // for each contour, display it in blue
                for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0])
                {
                    Imgproc.drawContours(output, contours, idx, new Scalar(250, 0, 0));

                    Rect rect = Imgproc.boundingRect(contours.get(idx));

                    rectList.add(rect);

                }
            }


            int pos = 0;
            if (rectList.size() > 0) {
                double largestArea = -1;
                Rect largestRect = new Rect();
                for (int i = 0; i < rectList.size(); i++) {
                    Rect checkRect = rectList.get(i);
                    if (checkRect.area() > largestArea /*&& Math.abs(checkRect.y + checkRect.height/2 - 135) < 10*/) {
                        largestArea = checkRect.area();
                        largestRect = checkRect;
                    }
                }
                Imgproc.rectangle(
                        output,
                        new Point(
                                largestRect.x,
                                largestRect.y),
                        new Point(
                                largestRect.x + largestRect.width,
                                largestRect.y + largestRect.height),
                        new Scalar(0, 255, 0), 4);

                int centerX = largestRect.x + largestRect.width/2;
                int centerY = largestRect.y + largestRect.height/2;
                telemetry.addData("Center X", centerX);
                telemetry.addData("Center Y", centerY);
                if (centerX <= 75)
                    pos  = 1;
                if (Math.abs(centerX-150) < 75)
                    pos = 2;
                if (centerX >= 225)
                    pos = 3;

                telemetry.addData("Position is", pos);
            }





            if (gamepad1.y)
                return mask;
            if (gamepad1.b)
                return morphOutput;
            if (gamepad1.x)
                return input;
            if (gamepad1.left_bumper)
                return blurredImage;
            if (gamepad1.right_bumper)
                return hsvImage;


            //Imgproc.line(output, new Point(0, 170), new Point(320, 170), new Scalar(255, 0, 0));
            //Imgproc.line(output, new Point(0, 70), new Point(320, 70), new Scalar(255, 0, 0));
            return output;

        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}