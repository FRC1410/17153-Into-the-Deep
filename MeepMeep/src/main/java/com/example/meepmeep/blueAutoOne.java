package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class blueAutoOne {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(730);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, 63, Math.toRadians(270)))
                        .forward(6)
                        .strafeLeft(50)
                        .turn(Math.toRadians(127))
                        .waitSeconds(1)
                        .strafeRight(15)
                        .turn(Math.toRadians(-90))
                        .waitSeconds(1)
                        .turn(Math.toRadians(90))
                        .strafeLeft(15)
                        .waitSeconds(1)
                        .strafeRight(15)
                        .turn(Math.toRadians(-160))
                        .waitSeconds(1)
                        .turn(Math.toRadians(160))
                        .strafeLeft(15)
                        .waitSeconds(1)
                        .strafeRight(15)
                        .turn(Math.toRadians(-130))
                        .waitSeconds(1)
                        .turn(Math.toRadians(130))
                        .strafeLeft(15)
                        .waitSeconds(1)
                        .turn(Math.toRadians(30))
                        .back(55)
                        .turn(Math.toRadians(120))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

