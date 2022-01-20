package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class T265 extends LinearOpMode {
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;

    @Override
    public void runOpMode() throws InterruptedException {
//        boolean justInitliazedT265 = false;
//        if (slamra == null) {
//            justInitliazedT265 = true;
//            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
//        } else {
//            slamra.stop();
//        }
//        slamra.start();
//        while(true){
//            T265Camera.CameraUpdate update = slamra.getLastReceivedCameraUpdate();
//            if(update == null || Math.abs(update.pose.getRotation().getDegrees()) < 0.001) {
//                sleep(50);
//            }else{
//                break;
//            }
//        }


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.useT265(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        drive.useT265(hardwareMap);
//        telemetry.addData("new localizer", drive.getLocalizer() != null);
        telemetry.update();

        waitForStart();
        drive.setPoseEstimate(new Pose2d());

        Localizer loc = ((LocalizerT265) drive.getLocalizer()).defaultLocalizer;
//        if(justInitliazedT265) {
//            slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(0, 0, new Rotation2d(0)));
//        }
        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x","%.2f", poseEstimate.getX());
            telemetry.addData("y","%.2f",poseEstimate.getY());
            telemetry.addData("h","%.2f",Math.toDegrees(poseEstimate.getHeading()));
            Pose2d pose2 = loc.getPoseEstimate();
            telemetry.addData("x2","%.2f",pose2.getX());
            telemetry.addData("y2","%.2f",pose2.getY());
            telemetry.addData("h2","%.2f",Math.toDegrees(pose2.getHeading()));

            if(slamra != null) {
                T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
                if (up == null) return;

                // We divide by 0.0254 to convert meters to inches
                Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
                Rotation2d rotation = up.pose.getRotation();

                telemetry.addData("x", translation.getX());
                telemetry.addData("y", translation.getY());
                telemetry.addData("h", rotation.getDegrees());

            }
            telemetry.update();
        }
        slamra.stop();
    }
}
