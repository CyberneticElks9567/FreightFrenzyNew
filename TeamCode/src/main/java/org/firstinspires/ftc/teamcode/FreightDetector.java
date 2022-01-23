package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class FreightDetector extends OpenCvPipeline {
    /**
     * Programmer:    Sean Pakros
     * Date Created:  12/15/21
     * Purpose: Code for detecting our shipping element using openCV.
     * This is based off of another teams code (Wolf Corp Robotics 12525)
     * for detecting skystones in a previous season which I then modified to work for this season and our shipping element.
     */
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        MIDDLE,
        RIGHT,
        LEFT
    }
    private Location location;

    static final Rect MIDDLE_ROI = new Rect(
            new Point(10, 80),
            new Point(80, 130));
    static final Rect RIGHT_ROI = new Rect(
            new Point(140, 80),
            new Point(210, 130));
    static double PERCENT_COLOR_THRESHOLD = 0.1;

    public FreightDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat middle = mat.submat(MIDDLE_ROI);
        Mat right = mat.submat(RIGHT_ROI);


        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        middle.release();
        right.release();

        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean stoneMiddle = middleValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (!stoneMiddle && !stoneRight) {
            location = Location.LEFT;
            telemetry.addData("ShippingElement Location", "left");
        } else if (stoneMiddle) {
            location = Location.RIGHT;
            telemetry.addData("ShippingElement Location", "middle");
        } else {
            location = Location.MIDDLE;
            telemetry.addData("ShippingElement Location", "right");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorEmpty = new Scalar(255, 0, 0);
        Scalar colorShipping = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE ? colorEmpty : colorShipping);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT ? colorEmpty : colorShipping);

        if (location == Location.LEFT) {
            Imgproc.rectangle(mat, MIDDLE_ROI, colorEmpty);
            Imgproc.rectangle(mat, RIGHT_ROI, colorEmpty);
        }
        return mat;
    }

    public Location getLocation() {
        return location;
    }
}