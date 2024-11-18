package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class testingClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(730);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive -> {
                    // Build the trajectory sequence
                    TrajectorySequenceBuilder sequenceBuilder = drive.trajectorySequenceBuilder(new Pose2d(0, 63, Math.toRadians(270)));
                    sequenceBuilder.forward(6)
                            //.turn(Math.toRadians(90))
                            //.forward(55)
                            .strafeLeft(50)
                            //.turn(Math.toRadians(45))
                            .waitSeconds(1)
                            //.turn(Math.toRadians(-148))
                            .turn(Math.toRadians(127))
                            .strafeRight(20);

                    return sequenceBuilder.build();
                });

        // Set MeepMeep background and start simulation
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    /**
     * Function to perform strafing and turning synchronously.
     * @param builder The trajectory sequence builder
     * @param distance The lateral distance to strafe
     * @param radians The angle in radians to turn
     */
    private static void strafeTurn(TrajectorySequenceBuilder builder, double distance, double radians) {
        // Assuming we start at Pose2d(x, y, heading), calculate the ending pose
        double xChange = distance * Math.cos(radians);  // Calculate lateral movement in x direction
        double yChange = distance * Math.sin(radians);  // Calculate lateral movement in y direction
        double finalHeading = radians;  // Final heading after the turn

        // Move to a new pose with adjusted position and heading
        builder.lineToLinearHeading(new Pose2d(xChange, yChange, finalHeading));
    }
}
