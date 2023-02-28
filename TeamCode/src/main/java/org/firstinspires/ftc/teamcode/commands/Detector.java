//FREIGHT FRENZY EXAMPLE
//Based on https://github.com/wolfcorpftc/SkystoneCVTutorial/blob/master/TeamCode/src/main/java/org/wolfcorp/cv/tutorial/SkystoneDetector.java
//https://www.youtube.com/watch?v=JO7dqzJi8lw&ab_channel=FTCteamWolfCorpRobotics12525
package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.*;
import org.firstinspires.ftc.teamcode.subsystems.Webcam;
import org.opencv.core.*;
import org.opencv.imgproc.*;
import org.openftc.easyopencv.*;

import java.util.*;

public class Detector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum TSEPosition{
        LEFT, MIDDLE, RIGHT
    }
    private TSEPosition position;

    static final Rect LEFT_ROI = new Rect(
            new Point(Webcam.WIDTH/20, Webcam.HEIGHT/20),
            new Point(Webcam.WIDTH/2-Webcam.WIDTH/20, Webcam.HEIGHT-Webcam.HEIGHT/20));
    static final Rect RIGHT_ROI = new Rect(
            new Point(Webcam.WIDTH-Webcam.WIDTH/20, Webcam.HEIGHT/20),
            new Point(Webcam.WIDTH/2+Webcam.WIDTH/20, Webcam.HEIGHT-Webcam.HEIGHT/20));
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public Detector(Telemetry t){
        telemetry = t;
    }
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(20, 100, 100); // lower bound HSV for yellow
        Scalar highHSV = new Scalar(30, 255, 255); // higher bound HSV for yellow

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean TSELeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean TSERight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (TSELeft) {
            position = TSEPosition.LEFT;
            telemetry.addData("TSE Position", "LEFT");
        }
        else if (TSERight) {
            position = TSEPosition.MIDDLE;
            telemetry.addData("TSE Position", "MIDDLE");
        }
        else {
            position = TSEPosition.RIGHT;
            telemetry.addData("TSE Position", "RIGHT");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar notHere = new Scalar(255, 0, 0);
        Scalar isHere = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, position == TSEPosition.LEFT? isHere:notHere);
        Imgproc.rectangle(mat, RIGHT_ROI, position == TSEPosition.MIDDLE? isHere:notHere);

        return mat;
    }

    public TSEPosition snapshotAnalysis(){
        return this.position;
    }
    public TSEPosition getPosition(){ return this.position;}
}
