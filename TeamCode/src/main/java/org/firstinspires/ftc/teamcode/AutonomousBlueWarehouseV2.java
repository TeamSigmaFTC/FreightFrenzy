package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Collection;

@Config
@Autonomous
public class AutonomousBlueWarehouseV2 extends LinearOpMode {

    private DcMotorEx foreforeArm;
    private DcMotorEx foreArm;
    private DcMotorEx backArm;
    private DcMotorEx intake;
    private TouchSensor magnet;
    private SampleMecanumDrive drive;

    private OpenCvCamera webcam;
    private ContourPipeline pipeline;

    private CRServo spinner;

    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;

    private int minRectangleArea = 2000;
    public static double leftBarcodeRangeBoundary = 0.3; //i.e 30% of the way across the frame from the left
    public static double rightBarcodeRangeBoundary = 0.7; //i.e 60% of the way across the frame from the left

    private int tsePos = 0;

    protected Pose2d startPose = new Pose2d(12, 63, Math.toRadians(-90));
    public static double SHIPPING_HUB_X = -5;
    public static double SHIPPING_HUB_Y = 57;
    public static double SHIPPING_HUB_ANGLE = 70;
    public static double TRANSITION_X = 15;
    public static double TRANSITION_Y = 66;
    public static double TRANSITION_ANGLE = 0;
    public static double WAREHOUSE_X = 43;
    public static double WAREHOUSE_Y = 66.5;
    public static double WAREHOUSE_ANGLE = 0;


    // Green Range                                      Y      Cr     Cb
//    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 0.0);
//    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0);
    // red range
//    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 170.0, 0.0);
//    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 150.0);
    // blue range
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 150);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 160.0, 255.0);
    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;
    private RevBlinkinLedDriver blinkinLedDriver;

    //hsvValues is an array that will hold the hue, saturation, and value information
    float hsvValues[] = {0F, 0F, 0F};

    @Override
    public void runOpMode() throws InterruptedException {

        magnet = hardwareMap.get(TouchSensor.class, "magnet");
        spinner = hardwareMap.get(CRServo.class, "spinner");
        foreforeArm = hardwareMap.get(DcMotorEx.class, "foreforearm");
        foreArm = hardwareMap.get(DcMotorEx.class, "forearm");
        backArm = hardwareMap.get(DcMotorEx.class, "backarm");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        sensorColor = hardwareMap.get(ColorSensor.class, "color");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        backArm.setTargetPositionTolerance(30);
        foreArm.setTargetPositionTolerance(30);
        backArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        foreArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        foreforeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline

        pipeline = new ContourPipeline(0, 0, 0.5, 0);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);

        webcam.setPipeline(pipeline);

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        drive = new SampleMecanumDrive(hardwareMap);
        drive.addT265(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(SHIPPING_HUB_X, SHIPPING_HUB_Y), Math.toRadians(SHIPPING_HUB_ANGLE))
//                .lineToLinearHeading(new Pose2d(SHIPPING_HUB_X, SHIPPING_HUB_Y, Math.toRadians(SHIPPING_HUB_ANGLE)))
                .build();
        TrajectorySequence trajseq3 = drive.trajectorySequenceBuilder((traj1.end()))
                .splineTo(new Vector2d(TRANSITION_X, TRANSITION_Y), Math.toRadians(TRANSITION_ANGLE))
                .splineTo(new Vector2d(WAREHOUSE_X, WAREHOUSE_Y), Math.toRadians(WAREHOUSE_ANGLE))
                .build();
        TrajectorySequence trajseq4 = drive.trajectorySequenceBuilder(trajseq3.end())
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        // limit max speed to 5in/s
                        return 8;
                    }
                })
                .addTemporalMarker(() -> intake.setPower(-1))
                .forward(4)
                .waitSeconds(0.4) // extra 0.5 seconds to make sure one is picked up
                .addTemporalMarker(() -> intake.setPower(0))
                .build();
        TrajectorySequence trajseq5 = drive.trajectorySequenceBuilder(trajseq4.end())
                .addTemporalMarker(() -> intake.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> intake.setPower(0))
                .strafeLeft(2)
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(5, trajseq4.end().getY() - 10), Math.toRadians(-120))
                .build();
        TrajectorySequence trajseq6 = drive.trajectorySequenceBuilder(trajseq5.end())
                .splineTo(new Vector2d(trajseq4.end().getX(), trajseq4.end().getY()), Math.toRadians(0))
                .build();

        if (magnet.isPressed()) {
            telemetry.addData("Initialized", "Arms in right position");
        } else {
            telemetry.addData("WARNING", "Arms NOT in position!");
        }
        telemetry.update();
        waitForStart();

        drive.setPoseEstimate(startPose);
        if (drive.t265 != null) {
            drive.t265.setPoseEstimate(startPose);
        } else if (drive.getLocalizer() instanceof LocalizerT265) {
            LocalizerT265 loc = (LocalizerT265) drive.getLocalizer();
            if (loc.defaultLocalizer != null) {
                loc.defaultLocalizer.setPoseEstimate(startPose);
            }
        }

        if (isStopRequested()) return;

        resetStartTime();
        resetStartTime();
        while (getRuntime() < 1) {
            if (pipeline.error) {
                telemetry.addData("Exception: ", pipeline.debug.getStackTrace());
            }

            double cameraWidth = pipeline.getCameraWidth();
            boolean foundRight = false;
            boolean foundLeft = false;
            boolean foundCenter = false;
            Collection<Rect> rects = pipeline.getRects();
            for(Rect rect: rects){
                double x = pipeline.getRectMidpointX(rect);
                if(x > rightBarcodeRangeBoundary * cameraWidth) {
                    foundRight = true;
                } else if (x < leftBarcodeRangeBoundary * cameraWidth) {
                    foundLeft = true;
                } else {
                    foundCenter = true;
                }
            }
            if(!foundLeft && foundCenter && foundRight) {
                telemetry.addData("Barcode Position", "Left");
                tsePos = 1;
                break;
            } else if (!foundCenter && foundLeft && foundRight) {
                telemetry.addData("Barcode Position", "Center");
                tsePos = 2;
                break;
            } else if (!foundRight && foundLeft && foundCenter) {
                telemetry.addData("Barcode Position", "Right");
                tsePos = 3;
                break;
            }
        }
        telemetry.update();

        //drive to TSH and drop freight
        drive.followTrajectory(traj1);

        int backArmDegree;
        int foreArmDegree;
        int foreforeArmDumpDegree;
        if (tsePos == 1) {
            backArmDegree = 225;
            foreArmDegree = 136;
            foreforeArmDumpDegree = 157;
            //bottom
        } else if (tsePos == 2) {
            backArmDegree = 219;
            foreArmDegree = 117;
            foreforeArmDumpDegree = 150;
            //mid
        } else {
            backArmDegree = 180;
            foreArmDegree = 135;
            foreforeArmDumpDegree = 200;
            //top
        }
        // start raising arm to drop position
        backArm.setTargetPosition(Common.backArmAngleToEncoder(backArmDegree));
        backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backArm.setVelocity(3500);
        foreArm.setTargetPosition(Common.foreArmAngleToEncoder(foreArmDegree));
        foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreArm.setVelocity(1200);
        foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(90));
        foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreforeArm.setVelocity(800);

        while (!Common.isInPosition(backArm)) {
            sleep(50);
        }
        backArm.setVelocity(0);
        while (!Common.isInPosition(foreArm)) {
            sleep(50);
        }
        foreArm.setVelocity(0);
        while (!Common.isInPosition(foreforeArm)) {
            sleep(50);
        }
        foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(foreforeArmDumpDegree));
        foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreforeArm.setVelocity(500);
        while (!Common.isInPosition(foreforeArm)) {
            sleep(50);
        }
        sleep(500);

        // drive to warehouse and put arm back in
//        drive.followTrajectory(traj2);
        followTrajectoryAndPutArmBackIn(trajseq3);
        int cycle = 0;

        while (cycle < 2) {
            cycle = cycle + 1;
            // inch forward to pickup freight
            drive.followTrajectorySequenceAsync(trajseq4);
            boolean intakeSuccess = false;

            while (drive.isBusy()) {
                drive.update();
                double distance = sensorDistance.getDistance(DistanceUnit.CM);
                Color.RGBToHSV(sensorColor.red(), sensorColor.green(), sensorColor.blue(), hsvValues);
                float hue = hsvValues[0];
                if (hue < 140 && hue > 75 && distance < 4) {
                    //show amber if the detected color is yellow-ish
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    intakeSuccess = true;
                } else if (hue > 154 && hue < 165 && distance < 3) {
                    //show white if the detected color is white-ish
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                    intakeSuccess = true;
                }
            }
            if (cycle == 2){
                break;
            }
            if (intakeSuccess) {
                // go back to shipping hub
                drive.followTrajectorySequence(trajseq5);
                // lift arm and dump
                backArm.setTargetPosition(Common.backArmAngleToEncoder(170));
                backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backArm.setVelocity(4000);

                while (!Common.isInPosition(backArm)) {
                    drive.update();
                }
                backArm.setVelocity(0);
                foreArm.setTargetPosition(Common.foreArmAngleToEncoder(129));
                foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreArm.setVelocity(1500);
                foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(90));
                foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreforeArm.setVelocity(800);
                while (!Common.isInPosition(foreforeArm)) {
                    drive.update();
                }
                foreforeArm.setVelocity(0);
                while (!Common.isInPosition(foreArm)) {
                    drive.update();
                }
                foreArm.setVelocity(0);
                foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(180));
                foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreforeArm.setVelocity(500);
                while (!Common.isInPosition(foreforeArm)) {
                    sleep(50);
                }

                // 2nd drive back to warehouse and put arm back in
                followTrajectoryAndPutArmBackIn(trajseq6);
            } else {
                intake.setPower(-0.5);
                sleep(500);
                intake.setPower(0);
                break;
            }
        }
        Storage.currentPose = drive.getPoseEstimate();
    }

    private void followTrajectoryAndPutArmBackIn(TrajectorySequence trajseq) {
        //park
        drive.followTrajectorySequenceAsync(trajseq);

        // put arm back in
        foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(90));
        foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreforeArm.setVelocity(700);
        backArm.setTargetPosition(Common.backArmAngleToEncoder(180));
        backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backArm.setVelocity(3000);

        while (!Common.isInPosition(foreforeArm) || !Common.isInPosition(backArm)) {
            drive.update();
        }
        foreforeArm.setVelocity(0);
        backArm.setVelocity(0);

        foreArm.setTargetPosition(Common.foreArmAngleToEncoder(310));
        foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreArm.setVelocity(3000);

        while (!Common.isInPosition(foreArm)) {
            drive.update();
        }
        foreArm.setVelocity(0);

        foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(-17));
        foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreforeArm.setVelocity(700);
        while (!Common.isInPosition(foreforeArm)) {
            drive.update();
        }

        foreArm.setTargetPosition(Common.foreArmAngleToEncoder(325));
        foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreArm.setVelocity(4000);
        backArm.setTargetPosition(Common.backArmAngleToEncoder(218));
        backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backArm.setVelocity(3000);

        while (!Common.isInPosition(foreArm)) {
            drive.update();
        }
        foreArm.setVelocity(0);
        while (!Common.isInPosition(backArm)) {
            drive.update();
        }
        backArm.setVelocity(0);
        // finish following trajseq
        while (drive.isBusy()) {
            drive.update();
        }

    }
}

