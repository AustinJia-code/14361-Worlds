package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import static java.lang.Math.*;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

//
@Autonomous(name = "\uD83D\uDDFF Left High \uD83D\uDDFF", group = "Final")
public class LeftHigh extends High {

    private double waitAtStorage = 0.2;
    private double waitAtScore = 0.1;
    public static Pose2d INIT = new Pose2d(-32.5, -65, toRadians(-90));
    public static Pose2d PARK_LEFT = new Pose2d(-58 , -16, toRadians(-90));
    public static Pose2d PARK_MIDDLE = new Pose2d(-35, -16, toRadians(-90));
    public static Pose2d PARK_RIGHT = new Pose2d(-8, -16, toRadians(-90));

    public void build(){
        SCORING_POSITION = new Pose2d(-27.25,-6, toRadians(45));
        STORAGE_POSITION = new Pose2d(-49.50, -15.1, toRadians(180));

        drive.setPoseEstimate(INIT);
        bot.claw.close();

        ScorePreload = drive.trajectorySequenceBuilder(INIT)
                .setConstraints(Constrainer.vel(50), Constrainer.accel(50))
                .addTemporalMarker(0.6, () -> bot.setPosition(State.HIGH))
                .addTemporalMarker(0.7, () -> bot.slide.setTarget(LinearSlides.spoolChange(1420)))
                .addTemporalMarker(0.8, () -> bot.arm.setPosition(State.HIGH))
                .addTemporalMarker(2.2, () -> {
                    bot.slide.incrementSlides(-1);
                    bot.arm.slamThatJawn();
                })
                .back(34)
                .splineTo(new Vector2d(SCORING_POSITION.getX()-1, SCORING_POSITION.getY()-3), SCORING_POSITION.getHeading())
                .resetConstraints()
                .build();
        WaitAtScore1 = waitSequence(ScorePreload, waitAtScore, false);

        ScoreToStorage1 = ScoreToStorage(ScorePreload, 1, 0, 0);
        WaitAtStorage1 = waitSequence(ScoreToStorage1, waitAtStorage, true);
        StorageToScore1 = StorageToScore(WaitAtStorage1, 0.35, 0, 0);
        WaitAtScore2 = waitSequence(StorageToScore1, waitAtScore, false);

        ScoreToStorage2 = ScoreToStorage(WaitAtScore2, 0, 0, 0);
        WaitAtStorage2 = waitSequence(ScoreToStorage2, waitAtStorage, true);
        StorageToScore2 = StorageToScore(WaitAtStorage2, 0, -0.35, 0);
        WaitAtScore3 = waitSequence(StorageToScore2, waitAtScore, false);

        ScoreToStorage3 = ScoreToStorage(WaitAtScore3, 0, 0, 0);
        WaitAtStorage3 = waitSequence(ScoreToStorage3, waitAtStorage, true);
        StorageToScore3 = StorageToScore(WaitAtStorage3, 0, -0.35, 0);
        WaitAtScore4 = waitSequence(StorageToScore3, waitAtScore, false);

        ScoreToStorage4 = ScoreToStorage(WaitAtScore4, 0.5, -0.25, 0);
        WaitAtStorage4 = waitSequence(ScoreToStorage4, waitAtStorage, true);
        StorageToScore4 = StorageToScore(WaitAtStorage4, 1, -0.4, 0);
        WaitAtScore5 = waitSequence(StorageToScore4, waitAtScore, false);

        ScoreToStorage5 = ScoreToStorage(WaitAtScore5, 1.5, -0.5, 0);
        WaitAtStorage5 = waitSequence(ScoreToStorage5, waitAtStorage, true);
        StorageToScore5 = StorageToScore(WaitAtStorage5, 1, -0.5, 0);
        WaitAtScore6 = waitSequence(StorageToScore5, waitAtScore, false);

        ParkMiddle = drive.trajectorySequenceBuilder(WaitAtScore6.end())
                .setReversed(true)
                .lineToLinearHeading(PARK_MIDDLE)
                .build();
        ParkLeft = drive.trajectorySequenceBuilder(WaitAtScore6.end())
                .setReversed(false)
                .lineToLinearHeading(PARK_LEFT)
                .build();
        ParkRight = drive.trajectorySequenceBuilder(WaitAtScore6.end())
                .setReversed(true)
                .lineToLinearHeading(PARK_MIDDLE)
                .strafeLeft(24)
                .build();
    }

    @Override
    public void setCameraPosition(){
        webcamName = "Right";
    }
}