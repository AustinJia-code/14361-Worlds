package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.Triangulator;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.*;
import org.opencv.core.Point;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.Core.max;
import static org.opencv.imgproc.Imgproc.*;

import com.qualcomm.robotcore.eventloop.opmode.*;

@Disabled
@TeleOp(name = "Camera Triangulation", group = "Final")
public class CameraTriangulation extends OpMode {
    float anglex;
    float angley;
    float botdist;
    float middist;
    float botbotang;
    float topbotang;
    float bottopang;
    float toptopang;
    float distbetween = (float) 2.95276;
    float tempang;
    float theirdLine;
    WebcamName top;
    WebcamName bottom;
    OpenCvSwitchableWebcam switchableWebcam;
    SamplePipeline pipe;
    int loop = 0;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        top = hardwareMap.get(WebcamName.class, "Top");
        bottom = hardwareMap.get(WebcamName.class, "Right");

        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, top, bottom);
        pipe = new SamplePipeline();

        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                switchableWebcam.setPipeline(pipe);
                switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    @Override
    public void loop() {
        float[] bottomNums = prossesImg(pipe.gimmeDat());
        float[] topNums = prossesImg(pipe.gimmeDat());
        switch(loop) {
            case 0:
                switchableWebcam.setActiveCamera(top);
                break;
            case 1:
                topNums = prossesImg(pipe.gimmeDat());
                break;
            case 2:
                switchableWebcam.setActiveCamera(bottom);
                break;
            case 3:
                bottomNums = prossesImg(pipe.gimmeDat());
                break;
            case 4:
                anglex = bottomNums[0];
                botbotang = bottomNums[1];
                bottopang = bottomNums[3];
                topbotang = topNums[1];
                toptopang = topNums[3];
                botdist = (float) ((distbetween * Math.tan(botbotang) * Math.tan(topbotang)) / (Math.tan(botbotang) + Math.tan(topbotang)));
                middist = (float) ((distbetween * Math.tan(bottopang) * Math.tan(toptopang)) / (Math.tan(bottopang) + Math.tan(toptopang)));
                tempang = bottopang - botbotang;
                theirdLine = (float) Math.sqrt((middist * middist) + (botdist * botdist) - (2 * middist * botdist) * Math.cos(tempang));
                angley = (float) (90 - (Math.acos(((theirdLine * theirdLine) + (botdist * botdist) - (middist * middist)) / (2 * theirdLine * botdist)) - botbotang));
                telemetry.addLine("x angle = " + anglex + "\ny angle = " + angley);
                telemetry.addLine("y offset = " + (-1 * (Math.sin(Math.toRadians(90 - angley)) * 33.5) + " in \nx offset = " + (Math.sin(Math.toRadians(90 - anglex)) * 33.5)) + " in");//positive x meals right and possitive y means up or further away

        }
        //output[0] = the x angle
        //output[1] = bot the angle from x=0
        //output[2]=bot the angle from y=0
        //output[3] = top the angle from x=0
        //output[4]=top the angle from y=0
    }

    public static float[] prossesImg(Mat pic) {
        Random rng = new Random(12345);
        float[] output = new float[5];
        //output[0] = the x angle
        //output[1] = bot the angle from x=0
        //output[2]=bot the angle from y=0
        //output[3] = top the angle from x=0
        //output[4]=top the angle from y=0
        float thoey = (float) ((pic.width() / 2) / Math.tan(27.5));
        Point center;
        Point bottom;
        inRange(pic, new Scalar(20, 100, 100), new Scalar(90, 255, 255), pic);
        GaussianBlur(pic, pic, new Size(3, 3), 3, 3);
        //Canny(pic, pic, 35, 125);
        Point test = findtl(pic);

        ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchey = new Mat();
        //Imgproc.findContours(pic,contours,hierarchey,Imgproc.RETR_TREE, CHAIN_APPROX_SIMPLE);
        //sets points
        Point tl = findtl(pic);
        Point tr = findtr(pic);
        Point bl = findbl(pic);
        Point br = findbr(pic);
        //some math
        output[0] = (float) (90 + Math.toDegrees(Math.atan((br.y - bl.y) / (br.x - bl.x))));
        center = new Point((tl.x + bl.x) / 2, (tl.y + bl.y) / 2);
        bottom = new Point((tr.x + br.x) / 2, (tr.y + br.y) / 2);
        output[1] = (float) Math.atan((bottom.x - (pic.width() / 2)) / thoey);
        output[2] = (float) Math.atan((bottom.y - (pic.height() / 2)) / thoey);//not acurate
        output[3] = (float) Math.atan((center.x - (pic.width() / 2)) / thoey);
        output[4] = (float) Math.atan((center.y - (pic.height() / 2)) / thoey);//not acurate

        //System.out.println(pic.height());
        //System.out.println(pic.width());
        //HighGui.imshow(""+pic,pic);
        //HighGui.waitKey();
        return output;
    }

    public static Mat loadImage(String imagePath) {
        Imgcodecs imageCodecs = new Imgcodecs();
        return imageCodecs.imread(imagePath);
    }

    public static Point findtl(Mat pic) {
        int top = 300;
        int left = 0;
        int bottom = 600;
        int right = pic.width();
        for (int x = left; x < right; x++) {
            for (int y = top; y < bottom; y++) {
                if (pic.get(y, x)[0] > 0) {
                    return new Point(x, y);
                }
            }
        }
        return null;
    }

    public static Point findtr(Mat pic) {
        int top = 300;
        int left = 0;
        int bottom = 600;
        int right = pic.width() - 600;
        for (int x = right; x > left; x--) {
            for (int y = top; y < bottom; y++) {
                if (pic.get(y, x)[0] > 0) {
                    return new Point(x, y);
                }
            }
        }
        return null;
    }

    public static Point findbl(Mat pic) {
        int top = 300;
        int left = 0;
        int bottom = 600;
        int right = pic.width();
        for (int x = left; x < right; x++) {
            for (int y = bottom; y > top; y--) {
                if (pic.get(y, x)[0] > 0) {
                    return new Point(x, y);
                }
            }
        }
        return null;
    }

    public static Point findbr(Mat pic) {
        int top = 300;
        int left = 0;
        int bottom = 600;
        int right = pic.width() - 200;
        for (int x = right; x > left; x--) {
            for (int y = bottom; y > top; y--) {
                if (pic.get(y, x)[0] > 0) {
                    return new Point(x, y);
                }
            }
        }
        return null;
    }

    class SamplePipeline extends OpenCvPipeline
    {
        Mat input;
        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);
            this.input = input;
            return input;
        }
        public Mat gimmeDat(){
            return input;
        }
    }
}