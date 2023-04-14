package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import static java.lang.Math.*;

import org.firstinspires.ftc.teamcode.commands.SleeveDetection.*;

//UNTUNED
@Autonomous(name = "ඞ Vent ඞ", group = "Final")
public class ParkOnly extends High {

    public static Pose2d INIT = new Pose2d(35, -65, toRadians(-90));
    public static Pose2d PARK_LEFT = new Pose2d(12, -36.5, toRadians(-90));
    public static Pose2d PARK_RIGHT = new Pose2d(60, -36.5, toRadians(-90));
    //public static Pose2d PARK_MIDDLE = new Pose2d(35, -35, toRadians(-90));

    @Override
    public void execute(TSEPosition position){
        camera.closeCameraDevice();

        drive.followTrajectorySequence(ParkMiddle);

        switch(position) {
            case LEFT:
                drive.followTrajectorySequence(ParkLeft);
                break;
            case RIGHT:
                drive.followTrajectorySequence(ParkRight);
                break;
        }
    }

    public void build(){
        drive.setPoseEstimate(INIT);

        ParkMiddle = drive.trajectorySequenceBuilder(INIT)
                .back(28.5)
                .build();
        ParkLeft = drive.trajectorySequenceBuilder(ParkMiddle.end())
                .setReversed(false)
                .lineToLinearHeading(PARK_LEFT)
                .build();
        ParkRight = drive.trajectorySequenceBuilder(ParkMiddle.end())
                .setReversed(false)
                .lineToLinearHeading(PARK_RIGHT)
                .build();
    }

    @Override
    public void setCameraPosition(){
        webcamName = "Right";
    }
}