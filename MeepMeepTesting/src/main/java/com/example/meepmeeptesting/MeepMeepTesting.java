package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        // width 338mm
        // height 442mm

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(
                        60,
                        60,
                        Math.toRadians(180),
                        Math.toRadians(180),
                        12
                )
                .setDimensions(13.3071, 17.4016)
                .build();

        bot.runAction(
                bot.getDrive()
                        // start on field wall
                        .actionBuilder(new Pose2d(62.5, -8, Math.toRadians(180)))
                        // starting load
                        .strafeToLinearHeading(new Vector2d(-8, -8), Math.toRadians(225))
                        //pickup 3rd spike
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(32, -34, Math.toRadians(-90)), Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(32, -60, Math.toRadians(-90)), Math.toRadians(-90))
                        //shoot
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-8, -8, Math.toRadians(225)), Math.toRadians(180))
//                        //goto corner -> pre-intake -> ram
//                        .strafeToLinearHeading(new Vector2d(55, -50), Math.toRadians(-90))
//                        .strafeToLinearHeading(new Vector2d(55, -65), Math.toRadians(-90))
//                        .strafeToLinearHeading(new Vector2d(55, -50), Math.toRadians(-90))
//                        .strafeToLinearHeading(new Vector2d(55, -65), Math.toRadians(-90))
//                        .strafeToLinearHeading(new Vector2d(55, -75), Math.toRadians(-90))
//                        // shoot
//                        .strafeToLinearHeading(new Vector2d(-8, -8), Math.toRadians(225))

//                        .strafeToLinearHeading(new Vector2d(36, -24 - 4), Math.toRadians(-90))
//                        .strafeToLinearHeading(new Vector2d(36, -48), Math.toRadians(-90))
//                        .strafeToLinearHeading(new Vector2d(-24, -24), Math.toRadians(225))

                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}