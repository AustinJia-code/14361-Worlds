package org.firstinspires.ftc.teamcode.opmodes.OldenCodes.autonomous;
//still tuning
import com.acmerobotics.roadrunner.geometry.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import static java.lang.Math.*;

import org.firstinspires.ftc.teamcode.commands.*;

@Autonomous(name = "Right Mid Shallow", group = "Final")
public class RoboPlayersRightMid extends ShallowMid {

    private double wait = 0.5;

    private double waitAtStorage = 0;
    private double waitAtScore = 0;
    public static Pose2d INIT = new Pose2d(35.5, -62.5, toRadians(-90));
    public static Pose2d PARK_LEFT = new Pose2d(13, -13, toRadians(180));
    public static Pose2d PARK_MIDDLE = new Pose2d(35.5, -10, toRadians(-90));
    public static Pose2d PARK_RIGHT = new Pose2d(60, -12.5, toRadians(0));

    public void build(){
        SCORING_POSITION = new Pose2d(32,-19.5, toRadians(200));
        STORAGE_POSITION = new Pose2d(52.5, -10.5, toRadians(0));

        drive.setPoseEstimate(INIT);

        ScorePreload = drive.trajectorySequenceBuilder(INIT)
                .setConstraints(Constrainer.vel(50), Constrainer.accel(50))
                .setTurnConstraint(Math.toRadians(120), Math.toRadians(120))
                .addTemporalMarker(1, () -> {
                    bot.setPosition(State.HIGH);
                })
                .addTemporalMarker(2, () -> {
                    bot.setPosition(State.MIDDLE);
                })
                .addTemporalMarker(5.4, () -> {
                    bot.arm.slamThatJawn();
                })
                .back(65)
                .forward(15)
                .turn(Math.toRadians(130))
                .back(8)
                .resetConstraints()
                .build();
        //WaitAtScore1 = waitSequence(ScorePreload, waitAtScore, false);

        ScoreToStorage1 = ScoreToStorage(ScorePreload, 0, 0, 0);

        WaitAtStorage1 = waitSequence(ScoreToStorage1, waitAtStorage, true);
        StorageToScore1 = StorageToScore(ScoreToStorage1, 0, 0, 0);
        //WaitAtScore2 = waitSequence(StorageToScore1, waitAtScore, false);

        ScoreToStorage2 = ScoreToStorage(StorageToScore1, 0.5, 0, 0);
        WaitAtStorage2 = waitSequence(ScoreToStorage2, waitAtStorage, true);
        StorageToScore2 = StorageToScore(ScoreToStorage2, 0, 0.5, 0);
        //WaitAtScore3 = waitSequence(StorageToScore2, waitAtScore, false);

        ScoreToStorage3 = ScoreToStorage(StorageToScore2, 0, 0.5, 0);
        WaitAtStorage3 = waitSequence(ScoreToStorage3, waitAtStorage, true);
        StorageToScore3 = StorageToScore(ScoreToStorage3, 0, 1.5, 0);
        //WaitAtScore4 = waitSequence(StorageToScore3, waitAtScore, false);

        ScoreToStorage4 = ScoreToStorage(StorageToScore3, 0, 1, 0);
        WaitAtStorage4 = waitSequence(ScoreToStorage4, waitAtStorage, true);
        StorageToScore4 = StorageToScore(ScoreToStorage4, -0.5, 1.5, 0);
        //WaitAtScore5 = waitSequence(StorageToScore4, waitAtScore, false);

        ScoreToStorage5 = ScoreToStorage(StorageToScore4, 0, 1, 0);
        WaitAtStorage5 = waitSequence(ScoreToStorage5, waitAtStorage, true);
        StorageToScore5 = StorageToScore(ScoreToStorage5, -0.75, 2.0, 0);
        //WaitAtScore6 = waitSequence(StorageToScore5, waitAtScore, false);

        ParkMiddle = drive.trajectorySequenceBuilder(StorageToScore5.end())
                .setReversed(false)
                .lineToLinearHeading(PARK_MIDDLE)
                .addTemporalMarker(0, () -> {
                    bot.arm.setPosition(State.LIFTED);
                    bot.claw.open();
                })
                .forward(12)
                .waitSeconds(1)
                .build();
        ParkLeft = drive.trajectorySequenceBuilder(StorageToScore5.end())
                .setReversed(false)
                .addTemporalMarker(0, () -> {
                    bot.arm.setPosition(State.LIFTED);
                    bot.claw.open();
                })
                .forward(6)
                .lineToLinearHeading(new Pose2d(PARK_MIDDLE.vec(), toRadians(0)))
                .back(23.5)
                .strafeRight(8)
                .waitSeconds(1)
                .build();
        ParkRight = drive.trajectorySequenceBuilder(StorageToScore5.end())
                .setReversed(false)
                .addTemporalMarker(0, () -> {
                    bot.arm.setPosition(State.LIFTED);
                    bot.claw.open();
                })
                .forward(6)
                .lineToLinearHeading(PARK_MIDDLE)
                .strafeLeft(28)
                .forward(8)
                .waitSeconds(1)
                .build();
    }

    @Override
    public void setCameraPosition(){
        webcamName = "Right";
    }
}