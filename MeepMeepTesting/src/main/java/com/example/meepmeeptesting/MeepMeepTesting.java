package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        Vector2d shooterVector = new Vector2d(-8,-55); //shooting vector

        MeepMeep mm = new MeepMeep(650, 60);
        mm
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_ULTIMATE_GOAL_DARK)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1.0f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(62, 62, Math.toRadians(210), Math.toRadians(210), 15.74)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-63, -56, Math.PI))
                                .setReversed(true)
                                .addTemporalMarker(0.1, () -> {})
                                .splineTo(new Vector2d(-8,-58), 0) //shooting
                                .waitSeconds(3.5) //total time for shooting sequence
                                .setReversed(false)

                                .lineToLinearHeading(new Pose2d(-15, -47, Math.toRadians(135)))
                                .forward(16,
                                        SampleMecanumDrive.getVelocityConstraint(5, 62, 15.74),
                                        SampleMecanumDrive.getAccelerationConstraint(62)
                                )
                                .waitSeconds(4.5)
                                .forward(12,
                                        SampleMecanumDrive.getVelocityConstraint(8, 62, 15.74),
                                        SampleMecanumDrive.getAccelerationConstraint(62)
                                )
                                .waitSeconds(4.5) //total time for shooting sequence
                                .setReversed(true)
                                .splineTo(new Vector2d(0, -59), 0) //wobble
                                .splineTo(new Vector2d(34, -59), 0) //wobble
                                .waitSeconds(2.5) //total time for wobble sequence
                                .lineToSplineHeading(new Pose2d(10,-56, Math.toRadians(90))) //park

                                .build()
                ).start();
    }
}