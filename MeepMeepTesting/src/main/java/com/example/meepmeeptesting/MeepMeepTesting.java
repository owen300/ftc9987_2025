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
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 6.781122045*2)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d((2*24)-6.78, (3*(24))-7.8, Math.toRadians(-90)))
                .splineTo(new Vector2d(8,36),Math.toRadians(-90))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(34,(1.4*(24))-7.8,Math.toRadians(0)),Math.toRadians(90))
                .splineToSplineHeading(new Pose2d((2.75*24)-6.78,(2.65*(24))-7.8,Math.toRadians(-125)),Math.toRadians(125))
                .splineToSplineHeading(new Pose2d(40,(1.4*(24))-7.8,Math.toRadians(0)),Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}