package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import static java.lang.Math.*;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

@Autonomous(name = "\uD83D\uDDFF Left Mid \uD83D\uDDFF", group = "Final")
public class LeftMid extends Mid {

    private double waitAtStorage = 0.1;
    private double waitAtScore = 0.1;
    public static Pose2d INIT = new Pose2d(-35.5, -62.5, toRadians(-90));
    public static Pose2d PARK_LEFT = new Pose2d(-60 , -15, toRadians(180));
    public static Pose2d PARK_MIDDLE = new Pose2d(-36, -18, toRadians(-90));
    public static Pose2d PARK_RIGHT = new Pose2d(-29, -15, toRadians(90));

    public void build(){
        SCORING_POSITION = new Pose2d(-27.75,-20.25, toRadians(-45));
        STORAGE_POSITION = new Pose2d(-50.5, -11.5, toRadians(180));

        drive.setPoseEstimate(INIT);

        ScorePreload = drive.trajectorySequenceBuilder(INIT)
                .setConstraints(Constrainer.vel(50), Constrainer.accel(50))
                .setTurnConstraint(Math.toRadians(120), Math.toRadians(120))
                .addTemporalMarker(0, () -> {
                    bot.setPosition(State.MIDDLE);
                })
                .addTemporalMarker(4.6, () -> {
                    bot.arm.slamThatJawn();
                })
                .back(50)
                .turn(Math.toRadians(-125))
                .back(8.5)
                .resetConstraints()
                .build();
        WaitAtScore1 = waitSequence(ScorePreload, waitAtScore, false);

        ScoreToStorage1 = ScoreToStorage(WaitAtScore1, 0, 0.5, 0);

        WaitAtStorage1 = waitSequence(ScoreToStorage1, waitAtStorage, true);
        StorageToScore1 = StorageToScore(WaitAtStorage1, 0, 0, 0);
        WaitAtScore2 = waitSequence(StorageToScore1, waitAtScore, false);

        ScoreToStorage2 = ScoreToStorage(WaitAtScore2, 0, -0.25, 0);
        WaitAtStorage2 = waitSequence(ScoreToStorage2, waitAtStorage, true);
        StorageToScore2 = StorageToScore(WaitAtStorage2, 0, -1, 0);
        WaitAtScore3 = waitSequence(StorageToScore2, waitAtScore, false);

        ScoreToStorage3 = ScoreToStorage(WaitAtScore3, 0, -1.5, 0);
        WaitAtStorage3 = waitSequence(ScoreToStorage3, waitAtStorage, true);
        StorageToScore3 = StorageToScore(WaitAtStorage3, -0.5, -3.0, 0);
        WaitAtScore4 = waitSequence(StorageToScore3, waitAtScore, false);

        ScoreToStorage4 = ScoreToStorage(WaitAtScore4, 0, -2, 0);
        WaitAtStorage4 = waitSequence(ScoreToStorage4, waitAtStorage, true);
        StorageToScore4 = StorageToScore(WaitAtStorage4, 0, -4.25, 0);
        WaitAtScore5 = waitSequence(StorageToScore4, waitAtScore, false);

        ScoreToStorage5 = ScoreToStorage(WaitAtScore5, 0, -3, 0);
        WaitAtStorage5 = waitSequence(ScoreToStorage5, waitAtStorage, true);
        StorageToScore5 = StorageToScore(WaitAtStorage5, 0, -6 , 0);
        WaitAtScore6 = waitSequence(StorageToScore5, waitAtScore, false);

        ParkMiddle = drive.trajectorySequenceBuilder(WaitAtScore6.end())
                .setReversed(false)
                .lineToLinearHeading(PARK_MIDDLE)
                .addTemporalMarker(0, () -> {
                    bot.arm.setPosition(State.LIFTED);
                    bot.claw.open();
                })
                .forward(8)
                .waitSeconds(1)
                .build();
        ParkLeft = drive.trajectorySequenceBuilder(WaitAtScore6.end())
                .setReversed(false)
                .addTemporalMarker(0, () -> {
                    bot.arm.setPosition(State.LIFTED);
                    bot.claw.open();
                })
                .lineToLinearHeading(PARK_MIDDLE)
                .strafeRight(24)
                .forward(8)
                .waitSeconds(1)
                .build();
        ParkRight = drive.trajectorySequenceBuilder(WaitAtScore6.end())
                .setReversed(false)
                .addTemporalMarker(0, () -> {
                    bot.arm.setPosition(State.LIFTED);
                    bot.claw.open();
                })
                .lineToLinearHeading(PARK_MIDDLE)
                .strafeLeft(26)
                .forward(8)
                .waitSeconds(1)
                .build();
    }

    @Override
    public void setCameraPosition(){
        webcamName = "Right";
    }
}