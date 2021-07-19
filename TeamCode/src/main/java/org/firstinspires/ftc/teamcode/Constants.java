package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.jetbrains.annotations.NotNull;

@Config
public class Constants {
    public enum Color {
        BLUE, RED
    }

    public enum Powershot {
        LEFT, MIDDLE, RIGHT
    }

    public enum BaseVelocity {
        HIGH,
        MIDDLE,
        POWERSHOT
    }

    public enum Side {
        IN,
        OUT,
        MIDDLE
    }

    public static double PI = Math.PI;
    public static double TAU = 2*PI;

    public static Pose2d START_OUT_RED = new Pose2d(-63.5, -14, 0);
    public static Pose2d START_OUT_BLUE = new Pose2d(-63.5, 14, 0);

    public static Pose2d START_IN_RED = new Pose2d(-63.5, -56, 0);
    public static Pose2d START_IN_BLUE = new Pose2d(-63.5, 56, 0);

    public static Pose2d START_MIDDLE_RED = new Pose2d(-63.5, -56, 0);
    public static Pose2d START_MIDDLE_BLUE = new Pose2d(-63.5, 56, 0);

    public static Vector2d RED_GOAL = new Vector2d(72, -36);
    public static Vector2d BLUE_GOAL = new Vector2d(72, 36);

    public static double SHOOTER_HIGH = 2100;
    public static double SHOOTER_MIDDLE = 1500;
    public static double SHOOTER_PS = 1550;

    public static double PIVOT_1_DROP = 0;
    public static double PIVOT_2_DROP = 0;

    public static double PIVOT_1_INIT = 0;
    public static double PIVOT_2_INIT = 0;

    public static double PIVOT_1_PICK = 0;
    public static double PIVOT_2_PICK = 0;

    public static double PIVOT_1_CARRY = 0;
    public static double PIVOT_2_CARRY = 0;



    public static Vector2d SHOOTER_OFFSET = new Vector2d(7, 3/25.4);

    public static double getVelocity(BaseVelocity velocity) {
        if(velocity.equals(BaseVelocity.HIGH)) {
            return SHOOTER_HIGH;
        }

        if(velocity.equals(BaseVelocity.MIDDLE)) {
            return SHOOTER_MIDDLE;
        }

        return SHOOTER_PS;
    }

    public static double getAngleToGoal(@NotNull Pose2d poseEstimate, Color color) {
        if(color.equals(Color.RED)) {
            return RED_GOAL.minus(poseEstimate.vec()).angle();
        }

        return BLUE_GOAL.minus(poseEstimate.vec()).angle();
    }

    public static double getDistanceToGoal(@NotNull Pose2d poseEstimate, Color color) {
        if(color.equals(Color.RED)) {
            return RED_GOAL.distTo(poseEstimate.vec());
        }

        return BLUE_GOAL.distTo(poseEstimate.vec());
    }

    public static void setStartPose(@NotNull SampleMecanumDrive drive, Color color, Side side) {
        switch(color) {
            case RED:
                if(side.equals(Side.IN)) drive.setPoseEstimate(START_IN_RED);
                if(side.equals(Side.OUT)) drive.setPoseEstimate(START_OUT_RED);
                if(side.equals(Side.MIDDLE)) drive.setPoseEstimate(START_MIDDLE_RED);
                break;
            case BLUE:
                if(side.equals(Side.IN)) drive.setPoseEstimate(START_IN_BLUE);
                if(side.equals(Side.OUT)) drive.setPoseEstimate(START_OUT_BLUE);
                if(side.equals(Side.MIDDLE)) drive.setPoseEstimate(START_MIDDLE_BLUE);
                break;
        }
    }

    public static Vector2d adjustPoseEstimate(@NotNull Pose2d poseEstimate) {
        return poseEstimate.vec().plus(SHOOTER_OFFSET);
    }

    public static double clipAngle(double angle, AngleUnit unit) {
        switch (unit) {
            case DEGREES:
                if(angle >= 180) angle -= 360;
                if(angle <= -180) angle += 360;
                break;
            case RADIANS:
                if(angle >= PI) angle -= TAU;
                if(angle <= -PI) angle += TAU;
                break;
        }

        return angle;
    }
}
