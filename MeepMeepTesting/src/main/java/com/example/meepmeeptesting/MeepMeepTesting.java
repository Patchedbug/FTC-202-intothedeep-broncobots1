package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;

import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15,15)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(38, 63, 0))
                .strafeTo(new Vector2d(56,63))
                .strafeTo(new Vector2d(32, 63))
                .strafeTo(new Vector2d(40, 15))
                .strafeTo(new Vector2d(44, 15))
                .strafeTo(new Vector2d(54, 59))
                .strafeTo(new Vector2d(54, 59))
                .strafeTo(new Vector2d(40, 48))
                .strafeTo(new Vector2d(47, 15))
                .strafeTo(new Vector2d(56, 15))
                .strafeTo(new Vector2d(56, 57))
                .strafeTo(new Vector2d(56, 15))
                .strafeTo(new Vector2d(63, 15))
                .strafeTo(new Vector2d(63, 57))
                .strafeTo(new Vector2d(25, 10))
                .splineTo(new Vector2d(-50,-50), Math.toRadians(60))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}