package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep).setDimensions(12.3, 18)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(310), Math.toRadians(310), 9.6)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(5, 53, Math.toRadians(55)))
                                        .lineToLinearHeading(new Pose2d(6, 65, Math.toRadians(0)))
                                        .lineTo(new Vector2d(40, 65))
                        .build()

//                                drive.trajectorySequenceBuilder(new Pose2d(-58, 58, Math.toRadians(-180)))
//                                        .setTangent(Math.toRadians(0))
//                                        .splineTo(new Vector2d(-36, 48),Math.toRadians(-45))
//                                        .build()
//                        drive.trajectorySequenceBuilder(new Pose2d(-36, 61.3, Math.toRadians(-90)))
//                                .splineTo(new Vector2d(-58, 58),Math.toRadians(-180) )
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}