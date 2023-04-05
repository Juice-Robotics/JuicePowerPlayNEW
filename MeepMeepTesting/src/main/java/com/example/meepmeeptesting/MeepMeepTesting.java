package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);
        Pose2d startPose = new Pose2d(in(92), in(165), rad(90));
        double x = 0.5;
        double y = 0.3;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,14)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 65, Math.toRadians(313.20662941325884) , Math.toRadians(313.20662941325884), 11.63)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .setReversed(true)
                                .back(42)
                                .splineTo(new Vector2d(7,17), Math.toRadians(135))
                                .waitSeconds(x)
                                .setReversed(false)
                                .splineTo(new Vector2d(23,10.5), Math.toRadians(0))
                                .forward(35)
                                .waitSeconds(y)
                                .setReversed(true)
                                .back(35)
                                .splineTo(new Vector2d(7,17), Math.toRadians(135))
                                .waitSeconds(x)
                                .setReversed(false)
                                .splineTo(new Vector2d(23,10.5), Math.toRadians(0))
                                .forward(35)
                                .waitSeconds(y)
                                .setReversed(true)
                                .back(35)
                                .splineTo(new Vector2d(7,17), Math.toRadians(135))
                                .waitSeconds(x)
                                .setReversed(false)
                                .splineTo(new Vector2d(23,10.5), Math.toRadians(0))
                                .forward(35)
                                .waitSeconds(y)
                                .setReversed(true)
                                .back(35)
                                .splineTo(new Vector2d(7,17), Math.toRadians(135))
                                .waitSeconds(x)
                                .setReversed(false)
                                .splineTo(new Vector2d(23,10.5), Math.toRadians(0))
                                .forward(35)
                                .waitSeconds(y)
                                .setReversed(true)
                                .back(35)
                                .splineTo(new Vector2d(7,17), Math.toRadians(135))
                                .waitSeconds(x)
                                .setReversed(false)
                                .splineTo(new Vector2d(23,10.5), Math.toRadians(0))
                                .forward(35)
                                .waitSeconds(y)
                                .setReversed(true)
                                .back(35)
                                .splineTo(new Vector2d(7,17), Math.toRadians(135))
                                .waitSeconds(x)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
    public static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double in(double centimeters) {
        return centimeters * 0.3837008;
    }
}