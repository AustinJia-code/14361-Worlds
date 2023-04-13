package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.*;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.bosch.*;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class TapeLocalizer implements Subsystem {
    private ColorRangeSensor one, two, three, four, five;
    private SampleMecanumDrive drive;
    public final double OFFSET = -1;
    public final double BLUE_THRESH = 90;
    public final double RED_THRESH = 100;

    private BNO055IMU imu;

    enum ALLIANCE{
        RED, BLUE
    }

    public TapeLocalizer(SampleMecanumDrive drive, HardwareMap hardwareMap) {
        this.drive = drive;

        one = hardwareMap.get(ColorRangeSensor.class, "one");
        two = hardwareMap.get(ColorRangeSensor.class, "two");
        //three = hardwareMap.get(ColorRangeSensor.class, "three");
        four = hardwareMap.get(ColorRangeSensor.class, "four");
        five = hardwareMap.get(ColorRangeSensor.class, "five");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public void relocalize(){
        double mean = 0;

        mean = relocalizeNeutral();

        Pose2d curPose = drive.getPoseEstimate();
        //drive.setPoseEstimate(new Pose2d(curPose.getX(), curPose.getY() + mean * OFFSET, Math.toRadians(-imu.getAngularOrientation().firstAngle)));
        drive.setPoseEstimate(new Pose2d(curPose.getX(), curPose.getY() + mean * OFFSET, curPose.getHeading()));
    }

    public double relocalizeNeutral(){
        double readings = 0;
        int readingCount = 0;

        if(one.red() > RED_THRESH || one.blue() > BLUE_THRESH){
            readings -= 2;
            readingCount++;
        }if(two.red() > RED_THRESH || two.blue() > BLUE_THRESH){
            readings -= 1;
            readingCount++;
        }

        if(four.red() > RED_THRESH || four.blue() > BLUE_THRESH){
            readings += 1;
            readingCount++;
        }if(five.red() > RED_THRESH || five.blue() > BLUE_THRESH){
            readings += 2;
            readingCount++;
        }

        if(readingCount > 0){
            return readings/readingCount;
        }
        return readings;
    }

    public double relocalizeRed(){
        double readings = 0;
        int readingCount = 0;

        if(one.red() > RED_THRESH){
            readings -= 2;
            readingCount++;
        }if(two.red() > RED_THRESH){
            readings -= 1;
            readingCount++;
        }

        if(four.red() > RED_THRESH){
            readings += 1;
            readingCount++;
        }if(five.red() > RED_THRESH){
            readings += 2;
            readingCount++;
        }

        if(readingCount > 0){
            return readings/readingCount;
        }
        return readings;
    }

    public double relocalizeBlue(){
        double readings = 0;
        int readingCount = 0;
        if(one.blue() > BLUE_THRESH){
            readings -= 2;
            readingCount++;
        }if(two.blue() > BLUE_THRESH){
            readings -= 1;
            readingCount++;
        }

        if(four.blue() > BLUE_THRESH){
            readings += 1;
            readingCount++;
        }if(five.blue() > BLUE_THRESH){
            readings += 2;
            readingCount++;
        }

        if(readingCount > 0){
            return readings/readingCount;
        }
        return readings;
    }
}