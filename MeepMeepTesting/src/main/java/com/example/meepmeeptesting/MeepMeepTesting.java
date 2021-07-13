package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        Vector2d shooterVector = new Vector2d(-8,-55); //shooting vector

        MeepMeep mm = new MeepMeep(650);
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
                        drive.trajectorySequenceBuilder(new Pose2d(-63.25, -15, 0))
                                .forward(50)
                                // drop arm
                                // open claw
                                // raise arm
                                .UNSTABLE_addTemporalMarkerOffset(0, ()->{})
                                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{})
                                .UNSTABLE_addTemporalMarkerOffset(1.2, ()->{})
                                .waitSeconds(2)

                                .back(50)
                                .build()
                ).start();
    }
}