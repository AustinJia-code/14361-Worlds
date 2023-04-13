package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import static java.lang.Math.*;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

//
@Autonomous(name = "\uD83D\uDC80 NO LOC \uD83D\uDDFF Left Bully \uD83D\uDDFF", group = "Final")
public class LeftBullyNoLoc extends High {

    private double waitAtStorage = 0.2;
    private double waitAtScore = 0.1;
    public static Pose2d INIT = new Pose2d(-33.5, -65, toRadians(-90));
    public static Pose2d PARK_LEFT = new Pose2d(-58 , -16, toRadians(-90));
    public static Pose2d PARK_MIDDLE = new Pose2d(-32, -16, toRadians(-90));
    public static Pose2d PARK_RIGHT = new Pose2d(-8, -16, toRadians(-90));

    public void build(){
        SCORING_POSITION = new Pose2d(-24,-3.25, toRadians(45));
        STORAGE_POSITION = new Pose2d(-47.9, -12.5, toRadians(180));

        drive.setPoseEstimate(INIT);

        ScorePreload = drive.trajectorySequenceBuilder(INIT)
                .setConstraints(Constrainer.vel(50), Constrainer.accel(50))
                .addTemporalMarker(0.6, () -> bot.setPosition(State.HIGH))
                .addTemporalMarker(0.7, () -> bot.slide.setTarget(LinearSlides.spoolChange(1420)))
                .addTemporalMarker(0.8, () -> bot.arm.setPosition(State.HIGH))
                .addTemporalMarker(2.2, () -> {
                    bot.slide.setPosition(State.LOW);
                    bot.arm.slamThatJawn();
                })
                .back(34)
                .splineTo(new Vector2d(SCORING_POSITION.getX()-2, SCORING_POSITION.getY()-4), SCORING_POSITION.getHeading())
                .back(2)
                .resetConstraints()
                .build();
        WaitAtScore1 = waitSequence(ScorePreload, waitAtScore, false);

        ScoreToStorage1 = ScoreToStorageNoLoc(ScorePreload, -0.2, -2, 0);
        WaitAtStorage1 = waitSequence(ScoreToStorage1, waitAtStorage, true);
        StorageToScore1 = StorageToScore(WaitAtStorage1, -0.75, -0.25, 0);
        WaitAtScore2 = waitSequence(StorageToScore1, waitAtScore, false);

        ScoreToStorage2 = ScoreToStorageNoLoc(WaitAtScore2, 0.25, -0.8, 0);
        WaitAtStorage2 = waitSequence(ScoreToStorage2, waitAtStorage, true);
        StorageToScore2 = StorageToScore(WaitAtStorage2, 0, -0.35, 0);
        WaitAtScore3 = waitSequence(StorageToScore2, waitAtScore, false);

        ScoreToStorage3 = ScoreToStorageNoLoc(WaitAtScore3, 0.7, -0.6, 0);
        WaitAtStorage3 = waitSequence(ScoreToStorage3, waitAtStorage, true);
        StorageToScore3 = StorageToScore(WaitAtStorage3, 0, -0.35, 0);
        WaitAtScore4 = waitSequence(StorageToScore3, waitAtScore, false);

        ScoreToStorage4 = ScoreToStorageNoLoc(WaitAtScore4, 0.3, -0.7, 0);
        WaitAtStorage4 = waitSequence(ScoreToStorage4, waitAtStorage, true);
        StorageToScore4 = StorageToScore(WaitAtStorage4, 0, -0.25, 0);
        WaitAtScore5 = waitSequence(StorageToScore4, waitAtScore, false);

        ScoreToStorage5 = ScoreToStorageNoLoc(WaitAtScore5, 1.6, -0.7, 0);
        WaitAtStorage5 = waitSequence(ScoreToStorage5, waitAtStorage, true);
        StorageToScore5 = StorageToScoreLast(WaitAtStorage5, 0, -0.25, 0);
        WaitAtScore6 = waitSequence(StorageToScore5, waitAtScore, false);

        ParkMiddle = drive.trajectorySequenceBuilder(WaitAtScore6.end())
                .setReversed(true)
                .addTemporalMarker(0, ()->{
                    bot.arm.setPosition(State.LIFTED);
                    bot.claw.open();
                })
                .lineToLinearHeading(PARK_MIDDLE)
                .forward(8)
                .waitSeconds(1)
                .build();
        ParkLeft = drive.trajectorySequenceBuilder(WaitAtScore6.end())
                .setReversed(false)
                .addTemporalMarker(0, ()->{
                    bot.arm.setPosition(State.LIFTED);
                    bot.claw.open();
                })
                .forward(6)
                .lineToLinearHeading(PARK_LEFT)
                .forward(8)
                .waitSeconds(1)
                .build();
        ParkRight = drive.trajectorySequenceBuilder(WaitAtScore6.end())
                .setReversed(true)
                .addTemporalMarker(0, ()->{
                    bot.arm.setPosition(State.LIFTED);
                    bot.claw.open();
                })
                .lineToLinearHeading(PARK_MIDDLE)
                .strafeLeft(27)
                .forward(8)
                .waitSeconds(1)
                .build();
    }

    @Override
    public void setCameraPosition(){
        webcamName = "Right";
    }
}