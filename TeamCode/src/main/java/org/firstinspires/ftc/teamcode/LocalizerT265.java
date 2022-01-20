package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.jetbrains.annotations.NotNull;

import kotlin.jvm.JvmOverloads;

public class LocalizerT265 implements Localizer {
    private Pose2d poseOffset = new Pose2d();
    private static Pose2d mPoseEstimate = new Pose2d();
    private Pose2d rawPose = new Pose2d();
    private T265Camera.CameraUpdate up;
    public Localizer defaultLocalizer;

    public static T265Camera slamra;

    private static T265Camera.PoseConfidence poseConfidence;

    public LocalizerT265(HardwareMap hardwareMap) {
        new LocalizerT265(hardwareMap, false);
    }

    public LocalizerT265(HardwareMap hardwareMap, boolean resetPos) {
        poseOffset = new Pose2d();
        mPoseEstimate = new Pose2d();
        rawPose = new Pose2d();

        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.95, hardwareMap.appContext);
            RobotLog.d("Created Realsense Object");
            setPoseEstimate(new Pose2d(0,0,0));
        }else{
            slamra.stop();
        }
        try {
            slamra.start();
        } catch (Exception ignored) {
            RobotLog.v("Realsense already started");
            if (resetPos) {
                slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(0,0, new Rotation2d(0)));
            }
        }
        if (slamra.getLastReceivedCameraUpdate().confidence == T265Camera.PoseConfidence.Failed) {
            RobotLog.e("Realsense Failed to get Position");
        }
    }

    /**
     * @return
     */
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        //variable up is updated in update()

        //The FTC265 library uses Ftclib geometry, so I need to convert that to road runner Geometry
        //TODO: convert all Ftclib geometry to ACME robotics geometry in T265Camera.java
        if (up != null) {
            Translation2d oldPose = up.pose.getTranslation();
            Rotation2d oldRot = up.pose.getRotation();
            //The T265's unit of measurement is meters.  dividing it by .0254 converts meters to inches.
            rawPose = new Pose2d(oldPose.getY() / .0254, -oldPose.getX() / .0254, norm(oldRot.getRadians() + angleModifer)); //raw pos
            mPoseEstimate = rawPose.plus(poseOffset); //offsets the pose to be what the pose estimate is;
        } else {
            RobotLog.v("NULL Camera Update");
        }

        return mPoseEstimate;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
//        RobotLog.v("Set Pose to " + pose2d.toString());
//        pose2d = new Pose2d(pose2d.getX(),pose2d.getY(),0);
//        RobotLog.v("SETTING POSE ESTIMATE TO " + pose2d.toString());
        poseOffset = pose2d.minus(rawPose);
//        poseOffset = new Pose2d(poseOffset.getX(), poseOffset.getY(), Math.toRadians(0));
//        RobotLog.v("SET POSE OFFSET TO " + poseOffset.toString());
    }

    public static T265Camera.PoseConfidence getConfidence() {
        return poseConfidence;
    }

    private static float angleModifer = 0;
    /**
     * @return the heading of the robot (in radains)
     */
    public static double getHeading() {
        return norma(mPoseEstimate.getHeading() - angleModifer);
    }

    /**
     * updates the camera.  Used in
     * @see org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive in update()
     */
    @Override
    public void update() {
        if(defaultLocalizer != null) {
            defaultLocalizer.update();
            Pose2d velocity = defaultLocalizer.getPoseVelocity();
            slamra.sendOdometry(velocity.getY() * 0.0254, -velocity.getX() * 0.0254);
        }
        up = slamra.getLastReceivedCameraUpdate();
        poseConfidence = up.confidence;
    }

    public void update(double X, double Y) {
        if(defaultLocalizer != null) {
            defaultLocalizer.update();
            Pose2d velocity = defaultLocalizer.getPoseVelocity();
            slamra.sendOdometry(velocity.getY() * 0.0254, -velocity.getX() * 0.0254);
        }
        up = slamra.getLastReceivedCameraUpdate();
        poseConfidence = up.confidence;
    }

    /**
     No idea what the purpose getPoseVelocity.  Everything works fine by just using getPoseEstimate()
     That said, the code to get the velocity is comment out below.  Haven't testing it much
     and I don't know how well getting the velocity work or if use the velocity has any effect
     at all.
     */
    @Override
    public Pose2d getPoseVelocity() {
        //variable up is updated in update()

        ChassisSpeeds velocity = up.velocity;
        return new Pose2d(velocity.vyMetersPerSecond /.0254,-velocity.vxMetersPerSecond /.0254,velocity.omegaRadiansPerSecond);
    }

    /**
     * @param angle angle in radians
     * @return normiazled angle between ranges 0 to 2Pi
     */
    private double norm(double angle)
    {
        while (angle>Math.toRadians(360)) angle-=Math.toRadians(360);
        while (angle<=0) angle+=Math.toRadians(360);
        return angle;
    }
    private static double norma(double angle)
    {
        while (angle>Math.toRadians(360)) angle-=Math.toRadians(360);
        while (angle<=0) angle+=Math.toRadians(360);
        return angle;
    }
}
