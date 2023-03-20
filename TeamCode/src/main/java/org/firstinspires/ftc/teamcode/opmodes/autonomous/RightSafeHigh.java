package org.firstinspires.ftc.teamcode.opmodes.autonomous;
//still tuning
import com.acmerobotics.roadrunner.geometry.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import static java.lang.Math.*;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

@Disabled
@Autonomous(name = "Right Safe High", group = "Final")
public class RightSafeHigh extends SafeHigh {

    private double wait = 0.5;

    private double waitAtStorage = 0.2;
    private double waitAtScore = 0.1;
    public static Pose2d INIT = new Pose2d(35.5, -62.5, toRadians(-90));
    public static Pose2d PARK_LEFT = new Pose2d(12 , -12, toRadians(-90));
    public static Pose2d PARK_MIDDLE = new Pose2d(35, -12, toRadians(-90));
    public static Pose2d PARK_RIGHT = new Pose2d(60, -12.5, toRadians(-90));

    public void build(){
        SCORING_POSITION = new Pose2d(6,-16.5, toRadians(225));
        STORAGE_POSITION = new Pose2d(25.5, -9, toRadians(0));

        drive.setPoseEstimate(INIT);
        bot.claw.close();

        ScorePreload = drive.trajectorySequenceBuilder(INIT)
                .addTemporalMarker(0, () -> bot.setPosition(State.HIGH))
                .addTemporalMarker(0, () -> bot.slide.setTarget(LinearSlides.spoolChange(1420)))
                .addTemporalMarker(2.6, () -> {
                    //bot.claw.outtakeUpdate(State.HIGH, gamepad1, gamepad2, 10);
                })
                .addTemporalMarker(2.8, () -> {
                    bot.slide.incrementSlides(-1);
                    bot.arm.slamThatJawn();
                })
                .back(39)
                .splineTo(new Vector2d(6.5 + 0.5,-18.75 + 0.25), Math.toRadians(215))
                .build();
        WaitAtScore1 = waitSequence(ScorePreload, waitAtScore);

        ScoreToStorage1 = ScoreToStorage(WaitAtScore1, 0, 0 ,0);

        WaitAtStorage1 = waitSequence(ScoreToStorage1, waitAtStorage);
        StorageToScore1 = StorageToScore(WaitAtStorage1, 0, 0.5, 0);
        WaitAtScore2 = waitSequence(StorageToScore1, waitAtScore);

        ScoreToStorage2 = ScoreToStorage(WaitAtScore2, 0, 0, 0);
        WaitAtStorage2 = waitSequence(ScoreToStorage2, waitAtStorage);
        StorageToScore2 = StorageToScore(WaitAtStorage2, 0, 0.5, 0);
        WaitAtScore3 = waitSequence(StorageToScore2, waitAtScore);

        ScoreToStorage3 = ScoreToStorage(WaitAtScore3, 0, 1, 0);
        WaitAtStorage3 = waitSequence(ScoreToStorage3, waitAtStorage);
        StorageToScore3 = StorageToScore(WaitAtStorage3, -1, 1.5, 0);
        WaitAtScore4 = waitSequence(StorageToScore3, waitAtScore);

        ScoreToStorage4 = ScoreToStorage(WaitAtScore4, 0, 1, 0);
        WaitAtStorage4 = waitSequence(ScoreToStorage4, waitAtStorage);
        StorageToScore4 = StorageToScore(WaitAtStorage4, -1, 1.5, 0);
        WaitAtScore5 = waitSequence(StorageToScore4, waitAtScore);

        ScoreToStorage5 = ScoreToStorage(WaitAtScore5, 0, 1, 0);
        WaitAtStorage5 = waitSequence(ScoreToStorage5, waitAtStorage);
        StorageToScore5 = StorageToScore(WaitAtStorage5, -.5, 1.6, 0);
        WaitAtScore6 = waitSequence(StorageToScore5, waitAtScore);

        ParkMiddle = drive.trajectorySequenceBuilder(WaitAtScore6.end())
                .setReversed(false)
                .splineTo(STORAGE_POSITION.vec(), 0)
                .addTemporalMarker(1, () -> {
                    bot.setPosition(State.INTAKING);
                })
                .build();
        ParkLeft = drive.trajectorySequenceBuilder(WaitAtScore6.end())
                .setReversed(false)
                .lineToLinearHeading(PARK_LEFT)
                .addTemporalMarker(1, () -> {
                    bot.setPosition(State.INTAKING);
                })
                .build();
        ParkRight = drive.trajectorySequenceBuilder(WaitAtScore6.end())
                .setReversed(false)
                .splineTo(STORAGE_POSITION.vec(), 0)
                .forward(24)
                .addTemporalMarker(1, () -> {
                    bot.setPosition(State.INTAKING);
                })
                .build();
    }

    @Override
    public void setCameraPosition(){
        webcamName = "Right";
    }
}