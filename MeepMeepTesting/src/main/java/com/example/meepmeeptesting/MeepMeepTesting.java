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
                        .actionBuilder(new Pose2d(-72 + 17.4016/2, -24, Math.toRadians(180)))
                        // starting load
                        .strafeToLinearHeading(new Vector2d(-24, -24), Math.toRadians(225))

                         // pickup 1
                        .strafeToLinearHeading(new Vector2d(-12, -24 - 4), Math.toRadians(-90))
                        .strafeToLinearHeading(new Vector2d(-12, -48), Math.toRadians(-90))

                        // open gate
                        .strafeToLinearHeading(new Vector2d(0, -56), Math.toRadians(180))
                        .waitSeconds(3)

                        // shoot 1
                        .strafeToLinearHeading(new Vector2d(-24, -24), Math.toRadians(225))


                        // pickup 2 -> shoot 2
                        .strafeToLinearHeading(new Vector2d(12, -24 - 4), Math.toRadians(-90))
                        .strafeToLinearHeading(new Vector2d(12, -48), Math.toRadians(-90))
                        .strafeToLinearHeading(new Vector2d(-24, -24), Math.toRadians(225))

                        .strafeToLinearHeading(new Vector2d(36, -24 - 4), Math.toRadians(-90))
                        .strafeToLinearHeading(new Vector2d(36, -48), Math.toRadians(-90))
                        .strafeToLinearHeading(new Vector2d(-24, -24), Math.toRadians(225))

                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}