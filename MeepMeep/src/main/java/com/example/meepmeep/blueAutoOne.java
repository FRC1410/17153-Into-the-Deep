package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.SampleMecanumDrive;
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
                        .turn(Math.toRadians(-53))
                        .waitSeconds(1)
                        .strafeLeft(15)
                        .turn(Math.toRadians(90))
                        .waitSeconds(1)
                        .turn(Math.toRadians(-90))
                        .strafeRight(15)
                        .waitSeconds(1)
                        .strafeLeft(15)
                        .turn(Math.toRadians(50))
                        .waitSeconds(1)
                        .turn(Math.toRadians(-50))
                        .strafeRight(15)
                        .waitSeconds(1)
                        .strafeLeft(15)
                        .turn(Math.toRadians(10))
                        .waitSeconds(1)
                        .turn(Math.toRadians(-10))
                        .strafeRight(15)
                        .waitSeconds(1)
                        .turn(Math.toRadians(30))
                        .forward(55)
                        .turn(Math.toRadians(-65))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
