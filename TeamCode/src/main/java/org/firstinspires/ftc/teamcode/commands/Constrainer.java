package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.*;

import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.trajectory.constraints.*;

public class Constrainer {
    public static TrajectoryVelocityConstraint vel(int vel) {
        return new TrajectoryVelocityConstraint() {
            @Override
            public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                return vel;
            }
        };
    }
    public static TrajectoryAccelerationConstraint accel(int accel) {
        return new TrajectoryAccelerationConstraint() {
            @Override
            public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                return accel;
            }
        };
    }
    public static AngularVelocityConstraint angVel(double accel) {
        return new AngularVelocityConstraint(accel);
    }
}
