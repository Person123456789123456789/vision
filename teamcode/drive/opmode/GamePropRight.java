package org.firstinspires.ftc.teamcode.drive.opmode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "GamePropRight", group = "1")
 public class GamePropRight extends LinearOpMode {

    OpenCvCamera webcam;
    private ElapsedTime runtime = new ElapsedTime();
    enum  GamePropLocation  {LEFT, MIDDLE, RIGHT};

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        webcam.setPipeline(new GamePropRight.SamplePipeline_Hue());

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.update();
            sleep(100);

            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

        }
    }

    //HUE Based Pipeline
    class SamplePipeline_Hue extends OpenCvPipeline
    {

        Scalar RED = new Scalar(255.0, 0.0, 0.0);
        Scalar BLUE = new Scalar(0.0, 0.0, 255.0);
        Scalar Green = new Scalar(0.0, 255.0, 0.0);

        //Point Left_pointA = new Point(20, 340.0);
        //Point Left_pointB = new Point(220, 680);

        Point Mid_pointA = new Point(290, 255);
        Point Mid_pointB = new Point(425,420);

        Point Right_pointA =  new Point(915, 375);
        Point Right_pointB =  new Point(1070, 450);
        //Mat left_Cb = null;
        Mat mid_Cb = null;
        Mat right_Cb = null;
        Mat HSV = new Mat();
        Mat hue = new Mat();
        int avg1, avg2, avg3;
        String position;

        boolean viewportPaused;

        public void inputToHue(Mat input) {
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(HSV, hue, 1); //Gets the HUE value out
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToHue(firstFrame);

            //left_Cb = hue.submat(new Rect(Left_pointA, Left_pointB));
            mid_Cb = hue.submat(new Rect(Mid_pointA, Mid_pointB));
            right_Cb = hue.submat(new Rect(Right_pointA, Right_pointB));

        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToHue(input);

           // avg1 = (int)Core.mean(left_Cb).val[0];
            avg2 = (int)Core.mean(mid_Cb).val[0];
            avg3 = (int)Core.mean(right_Cb).val[0];

            avg1 = Math.abs(avg2-avg3);


            // double maxOneTwo = Math.max(avg1, avg2);
            //double max = Math.max(maxOneTwo, avg3);
            double max = Math.max(avg2,avg3);
            /*Imgproc.rectangle(
                    input,
                    Left_pointA,
                    Left_pointB,
                    RED, 4);*/
            Imgproc.rectangle(
                    input,
                    Mid_pointA,
                    Mid_pointB,
                    RED, 4);
                   Imgproc.rectangle(
                            input,
                            Right_pointA,
                            Right_pointB,
                            RED, 4);

            if (avg1 < 50) {
                position = "Left";
            } else if (max == avg2) {
                Imgproc.rectangle(
                        input,
                        Mid_pointA,
                        Mid_pointB,
                        Green);
                position = "Middle";
            } else {
                        Imgproc.rectangle(
                                input,
                                Right_pointA,
                                Right_pointB,
                                Green);
                        position = "Right";
                    }



            telemetry.addData("[avg1]", avg1);
            telemetry.addData("[avg2]", avg2);
            telemetry.addData("[avg3]", avg3);
            telemetry.addData("Position", position);

            telemetry.update();
            return input;
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

    //CB Based Pipeline -- Can be Used for Blue
    class SamplePipeline_CB extends OpenCvPipeline
    {

        Scalar RED = new Scalar(255.0, 0.0, 0.0);
        Scalar BLUE = new Scalar(0.0, 0.0, 255.0);
        Scalar Green = new Scalar(0.0, 255.0, 0.0);

        //Point Left_pointA = new Point(0, 200.0);
        //Point Left_pointB = new Point(150, 400);

        Point Mid_pointA = new Point(290, 255);
        Point Mid_pointB = new Point(425,420);

        Point Right_pointA =  new Point(915, 375);
        Point Right_pointB =  new Point(1070, 450);

        //Mat left_Cb = null;
        Mat mid_Cb = null;
        Mat right_Cb = null;
        Mat YCrCb = new Mat();
        Mat cb = new Mat();
        int avg1, avg2, avg3;

        boolean viewportPaused;

        public void inputToHue(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, cb, 2); //Cb gives you difference from the Blue value
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToHue(firstFrame);

            //left_Cb = cb.submat(new Rect(Left_pointA, Left_pointB));
            mid_Cb = cb.submat(new Rect(Mid_pointA, Mid_pointB));
            right_Cb = cb.submat(new Rect(Right_pointA, Right_pointB));

        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToHue(input);

            //avg1 = (int)Core.mean(left_Cb).val[0];
            avg2 = (int)Core.mean(mid_Cb).val[0];
            avg2 = (int)Core.mean(right_Cb).val[0];

            double minOneTwo = Math.min(avg1, avg2);
            double min = Math.min(minOneTwo, avg3);

            /*Imgproc.rectangle(
                    input,
                    Left_pointA,
                    Left_pointB,
                    RED, 4);*/
            Imgproc.rectangle(
                    input,
                    Mid_pointA,
                    Mid_pointB,
                    RED, 4);
                Imgproc.rectangle(
                        input,
                        Right_pointA,
                        Right_pointB,
                        RED, 4);

            /*if(min == avg1) {
                Imgproc.rectangle(
                        input,
                        Left_pointA,
                        Left_pointB,
                        Green);
            }*/ if (min == avg2) {
                Imgproc.rectangle(
                        input,
                        Mid_pointA,
                        Mid_pointB,
                        Green);
            } else {
                    Imgproc.rectangle(
                            input,
                            Right_pointA,
                            Right_pointB,
                            Green);
                }



            telemetry.addData("[avg1]", avg1);
            telemetry.addData("[avg2]", avg2);
            telemetry.addData("[avg3]", avg3);

            telemetry.update();
            return input;
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

    //CR Based Pipeline -- Can be Used for RED
    class SamplePipeline_CR extends OpenCvPipeline
    {

        Scalar RED = new Scalar(255.0, 0.0, 0.0);
        Scalar BLUE = new Scalar(0.0, 0.0, 255.0);
        Scalar Green = new Scalar(0.0, 255.0, 0.0);

       // Point Left_pointA = new Point(275, 225.0);
        //Point Left_pointB = new Point(550, 500);

        Point Mid_pointA = new Point(290, 255);
        Point Mid_pointB = new Point(425,420);

        Point Right_pointA =  new Point(915, 375);
        Point Right_pointB =  new Point(1070, 450);

       // Mat left_Cb = null;
        Mat mid_Cb = null;
        Mat right_Cb = null;
        Mat YCrCb = new Mat();
        Mat cb = new Mat();
        int avg1, avg2, avg3;

        boolean viewportPaused;

        public void inputToHue(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, cb, 1); //Cr gives you difference from the RED value
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToHue(firstFrame);

           // left_Cb = cb.submat(new Rect(Left_pointA, Left_pointB));
            mid_Cb = cb.submat(new Rect(Mid_pointA, Mid_pointB));
            right_Cb = cb.submat(new Rect(Right_pointA, Right_pointB));

        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToHue(input);

            //avg1 = (int)Core.mean(left_Cb).val[0];
            avg2 = (int)Core.mean(mid_Cb).val[0];
            avg3 = (int)Core.mean(right_Cb).val[0];

            double minOneTwo = Math.min(avg1, avg2);
            double min = Math.min(minOneTwo, avg3);

            /*Imgproc.rectangle(
                    input,
                    Left_pointA,
                    Left_pointB,
                    RED, 4);*/
            Imgproc.rectangle(
                    input,
                    Mid_pointA,
                    Mid_pointB,
                    RED, 4);
            Imgproc.rectangle(
                    input,
                    Right_pointA,
                    Right_pointB,
                    RED, 4);

            /*if(min == avg1) {
                Imgproc.rectangle(
                        input,
                        Left_pointA,
                        Left_pointB,
                        Green)
            }*/ if (min == avg2) {
                Imgproc.rectangle(
                        input,
                        Mid_pointA,
                        Mid_pointB,
                        Green);
            } else {
                    Imgproc.rectangle(
                            input,
                            Right_pointA,
                            Right_pointB,
                            Green);
                }



            telemetry.addData("[avg1]", avg1);
            telemetry.addData("[avg2]", avg2);
            telemetry.addData("[avg3]", avg3);

            telemetry.update();
            return input;
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

