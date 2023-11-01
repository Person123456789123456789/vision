package org.firstinspires.ftc.teamcode.drive.Auto;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class GamePropRight extends OpenCvPipeline{

    enum gamePropPosition {
        LEFT, CENTER, RIGHT
    }

    TelemetryImpl telemetry;

    Point Mid_pointA = new Point(290, 255);
    Point Mid_pointB = new Point(425,420);

    Point Right_pointA =  new Point(915, 375);
    Point Right_pointB =  new Point(1070, 450);

    public static gamePropPosition position = gamePropPosition.LEFT; //Default Position

    Mat mid_Hue = null;
    Mat right_Hue = null;
    Mat HSV = new Mat();
    Mat hue = new Mat();
    int avg1, avg2, avg3;
    Scalar RED = new Scalar(255.0, 0.0, 0.0);
    Scalar BLUE = new Scalar(0.0, 0.0, 255.0);
    public void inputToHue(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(HSV, hue, 1); //Gets the HUE value out
    }

    @Override
    public void init(Mat firstFrame)
    {
        inputToHue(firstFrame);

        mid_Hue = hue.submat(new Rect(Mid_pointA, Mid_pointB));
        right_Hue = hue.submat(new Rect(Right_pointA, Right_pointB));
    }
    @Override
    public Mat processFrame(Mat input)
    {
        inputToHue(input);

        // avg1 = (int)Core.mean(left_Cb).val[0];
        avg2 = (int)Core.mean(mid_Hue).val[0];
        avg3 = (int)Core.mean(right_Hue).val[0];

        avg1 = Math.abs(avg2-avg3);
        int max = Math.max(avg2,avg3);

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
            position = gamePropPosition.LEFT;
        } else if (max == avg2) {
            position = gamePropPosition.CENTER;
        } else {
            position = gamePropPosition.RIGHT;
        }

       // telemetry.addData("[avg1]", avg1);
       // telemetry.addData("[avg2]", avg2);
       // telemetry.addData("[avg3]", avg3);
        //telemetry.addData("Position", position);

        //telemetry.update();
        return input;
    }
}