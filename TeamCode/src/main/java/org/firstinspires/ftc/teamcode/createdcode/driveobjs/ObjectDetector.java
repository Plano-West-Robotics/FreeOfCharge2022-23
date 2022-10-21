package org.firstinspires.ftc.teamcode.createdcode.driveobjs;

import static org.firstinspires.ftc.teamcode.createdcode.configs.AutoConfig.MAX_H;
import static org.firstinspires.ftc.teamcode.createdcode.configs.AutoConfig.MAX_S;
import static org.firstinspires.ftc.teamcode.createdcode.configs.AutoConfig.MAX_V;
import static org.firstinspires.ftc.teamcode.createdcode.configs.AutoConfig.MIN_H;
import static org.firstinspires.ftc.teamcode.createdcode.configs.AutoConfig.MIN_S;
import static org.firstinspires.ftc.teamcode.createdcode.configs.AutoConfig.MIN_V;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import java.util.concurrent.TimeUnit;

@Config
public class ObjectDetector {

    private OpenCvWebcam webcam;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private final HardwareMap hardwareMap;
    private int pos;

    private final int approxHeight;
    private final int pos1;
    private final int pos2;
    private final int pos3;

    public ObjectDetector(HardwareMap hardwareMap, int approxHeight, int pos1, int pos2, int pos3) {
        this.hardwareMap = hardwareMap;
        startCamera();
        this.pos1 = pos1;
        this.pos2 = pos2;
        this.pos3 = pos3;
        this.approxHeight = approxHeight;

    }

    public void startCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new DetectionPipeline());

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        for (long stop = System.nanoTime() + TimeUnit.SECONDS.toNanos(4); stop > System.nanoTime(); ) {
            dashboard.startCameraStream(webcam, 0);


            dashboardTelemetry.addData("Frame Count", webcam.getFrameCount());
            dashboardTelemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            dashboardTelemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            dashboardTelemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            dashboardTelemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            dashboardTelemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            dashboardTelemetry.update();


        }
        webcam.stopStreaming();

    }

    public int getPos() {
        return pos;
    }

    public void endStream() {
        webcam.stopStreaming();
    }


    class DetectionPipeline extends OpenCvPipeline {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input) {
            Mat blurredImage = new Mat();
            Mat hsvImage = new Mat();
            Mat mask = new Mat();
            Mat morphOutput = new Mat();

            // remove some noise
            Imgproc.blur(input, blurredImage, new Size(7, 7));

            // convert the frame to HSV
            Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);


            Scalar minValues = new Scalar(MIN_H, MIN_S, MIN_V);
            Scalar maxValues = new Scalar(MAX_H, MAX_S, MAX_V);
            Core.inRange(hsvImage, minValues, maxValues, mask);

            //erodes the little things

            Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
            Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));

            Imgproc.erode(mask, morphOutput, erodeElement);

            Imgproc.dilate(mask, morphOutput, dilateElement);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            Mat output = input.clone();

            List<Rect> rectList = new ArrayList<>();

            // find contours
            Imgproc.findContours(morphOutput, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
            if (hierarchy.size().height > 0 && hierarchy.size().width > 0) {
                // for each contour, display it in blue
                for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0]) {
                    Imgproc.drawContours(output, contours, idx, new Scalar(250, 0, 0));

                    Rect rect = Imgproc.boundingRect(contours.get(idx));
                    rectList.add(rect);

                }
            }


            if (rectList.size() > 0) {
                double largestArea = -1;
                Rect largestRect = new Rect();
                for (int i = 0; i < rectList.size(); i++) {
                    Rect checkRect = rectList.get(i);
                    if (checkRect.area() > largestArea && Math.abs(checkRect.y + checkRect.height / 2 - approxHeight) < 10) {
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

                int centerX = largestRect.x + largestRect.width / 2;
                int centerY = largestRect.y + largestRect.height / 2;

                if (Math.abs(pos1 - centerX) < Math.abs(pos2 - centerX) && Math.abs(pos1 - centerX) < Math.abs(pos3 - centerX))
                    pos = 1;
                else if (Math.abs(pos2 - centerX) < Math.abs(pos1 - centerX) && Math.abs(pos2 - centerX) < Math.abs(pos3 - centerX))
                    pos = 2;
                else if (Math.abs(pos3 - centerX) < Math.abs(pos2 - centerX) && Math.abs(pos3 - centerX) < Math.abs(pos1 - centerX))
                    pos = 3;


            }
            dashboardTelemetry.addData("pos", pos);

            //Imgproc.line(output, new Point(0, 170), new Point(320, 170), new Scalar(255, 0, 0));
            Imgproc.line(output, new Point(0, 70), new Point(320, 70), new Scalar(255, 0, 0));

            return output;
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
    }
}