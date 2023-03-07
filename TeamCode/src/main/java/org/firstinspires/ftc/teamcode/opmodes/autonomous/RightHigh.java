package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import static java.lang.Math.*;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

//TUNED, 6.75 setup
@Autonomous(name = "\uD83D\uDC17 Right High \uD83D\uDC17", group = "Final")
public class RightHigh extends High {

    private double waitAtStorage = 0.2;
    private double waitAtScore = 0.1;
    public static Pose2d INIT = new Pose2d(31, -65, toRadians(-90));
    public static Pose2d PARK_LEFT = new Pose2d(9, -12, toRadians(-90));
    public static Pose2d PARK_MIDDLE = new Pose2d(30, -11, toRadians(-90));
    public static Pose2d PARK_RIGHT = new Pose2d(58, -12, toRadians(0));

    public void build(){
        SCORING_POSITION = new Pose2d(23.9,-9.6, toRadians(135));
        STORAGE_POSITION = new Pose2d(56, -12, toRadians(0));

        drive.setPoseEstimate(INIT);
        bot.claw.close();

        ScorePreload = drive.trajectorySequenceBuilder(INIT)
                .addTemporalMarker(0, () -> bot.setPosition(State.HIGH))
                .addTemporalMarker(0.2, () -> bot.slide.setTarget(LinearSlides.spoolChange(1420)))
                .addTemporalMarker(2.0, () -> {
                    bot.claw.outtakeUpdate(State.HIGH, gamepad1, gamepad2, 10);
                })
                .addTemporalMarker(2.2, () -> {
                    bot.slide.incrementSlides(-1);
                    bot.arm.slamThatJawn();
                })
                .back(34)
                .splineTo(new Vector2d(SCORING_POSITION.getX()-0.75, SCORING_POSITION.getY()+1), SCORING_POSITION.getHeading())
                .build();
        WaitAtScore1 = waitSequence(ScorePreload, waitAtScore);

        ScoreToStorage1 = ScoreToStorage(ScorePreload, -1, -1, 0);
        WaitAtStorage1 = waitSequence(ScoreToStorage1, waitAtStorage);
        StorageToScore1 = StorageToScore(WaitAtStorage1, 0.9, 3.8, 0);
        WaitAtScore2 = waitSequence(StorageToScore1, waitAtScore);

        ScoreToStorage2 = ScoreToStorage(WaitAtScore2, -0.5, -0.5, 0);
        WaitAtStorage2 = waitSequence(ScoreToStorage2, waitAtStorage);
        StorageToScore2 = StorageToScore(WaitAtStorage2, 0.5, 4.0 , 0);
        WaitAtScore3 = waitSequence(StorageToScore2, waitAtScore);

        ScoreToStorage3 = ScoreToStorage(WaitAtScore3, -0.55, 0, 0);
        WaitAtStorage3 = waitSequence(ScoreToStorage3, waitAtStorage);
        StorageToScore3 = StorageToScore(WaitAtStorage3, 0.25, 5.0, 0);
        WaitAtScore4 = waitSequence(StorageToScore3, waitAtScore);

        ScoreToStorage4 = ScoreToStorage(WaitAtScore4, -1.25, 1.5, 0);
        WaitAtStorage4 = waitSequence(ScoreToStorage4, waitAtStorage);
        StorageToScore4 = StorageToScore(WaitAtStorage4, 0.25, 5.0, 0);
        WaitAtScore5 = waitSequence(StorageToScore4, waitAtScore);

        ScoreToStorage5 = ScoreToStorage(WaitAtScore5, -1.25, 2.5, 0);
        WaitAtStorage5 = waitSequence(ScoreToStorage5, waitAtStorage);
        StorageToScore5 = StorageToScore(WaitAtStorage5, -0.5, 6.4 , 0);
        WaitAtScore6 = waitSequence(StorageToScore5, waitAtScore);

        ParkMiddle = drive.trajectorySequenceBuilder(WaitAtScore6.end())
                .setReversed(true)
                .lineToLinearHeading(PARK_MIDDLE)
                .build();
        ParkLeft = drive.trajectorySequenceBuilder(WaitAtScore6.end())
                .setReversed(false)
                .lineToLinearHeading(PARK_MIDDLE)
                .strafeRight(24)
                .build();
        ParkRight = drive.trajectorySequenceBuilder(WaitAtScore6.end())
                .setReversed(true)
                .lineToLinearHeading(PARK_RIGHT)
                .build();
    }

    @Override
    public void setCameraPosition(){
        webcamName = "Right";
    }
}