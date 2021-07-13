package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Sequences {
    Robot robot;
    TrajectorySequence basicGo;
    Pose2d startPose;

    public Sequences(Robot robot) {
        this.robot = robot;
        startPose = new Pose2d();
    }

    public Sequences(Robot robot, Pose2d pose2d) {
        this.robot = robot;
        startPose = pose2d;
    }

    public void generateSequences() {
        basicGo = robot.drive.trajectorySequenceBuilder(startPose)
                .forward(30)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.wobble.openClaw())
                .UNSTABLE_addTemporalMarkerOffset(1, () -> robot.wobble.closeClaw())
                .back(30)
                .build();
    }
}
