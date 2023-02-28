package org.firstinspires.ftc.teamcode.commands;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Triangulator extends OpenCvPipeline {
    Mat mat = new Mat();
    @Override
    public Mat processFrame(Mat input) {
        mat = input;
        return input;
    }

    public Mat gimme(){
        return mat;
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Mat rgba = inputFrame.rgba();
        return rgba;
    }
}