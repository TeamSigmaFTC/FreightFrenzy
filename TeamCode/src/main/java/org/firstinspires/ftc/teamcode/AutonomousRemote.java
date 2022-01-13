package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous
public class AutonomousRemote extends LinearOpMode {

    private DcMotorEx foreforeArm;
    private DcMotorEx foreArm;
    private DcMotorEx backArm;
    private DcMotorEx intake;
    private TouchSensor magnet;

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

    protected Pose2d startPose = new Pose2d(-36, 63, Math.toRadians(-90));
    public static double SPINNER_X = -57;
    public static double SPINNER_Y = 58;
    public static double SPINNER_ANGLE = -180;
    public static double SHIPPING_HUB_X = -36;
    public static double SHIPPING_HUB_Y = 48;
    public static double SHIPPING_HUB_ANGLE = -45;
    public static double TRANSITION_X = 12;
    public static double TRANSITION_Y = 65.5;
    public static double TRANSITION_ANGLE = 0;
    public static double WAREHOUSE_X = 40;
    public static double WAREHOUSE_Y = 65;
    public static double WAREHOUSE_ANGLE = 0;

    // Green Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0);


    @Override
    public void runOpMode() throws InterruptedException {

        magnet = hardwareMap.get(TouchSensor.class, "magnet");
        spinner = hardwareMap.get(CRServo.class, "spinner");
        foreforeArm = hardwareMap.get(DcMotorEx.class, "foreforearm");
        foreArm = hardwareMap.get(DcMotorEx.class, "forearm");
        backArm = hardwareMap.get(DcMotorEx.class, "backarm");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        backArm.setTargetPositionTolerance(30);
        foreArm.setTargetPositionTolerance(30);
        backArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        foreArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        foreforeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline

        pipeline = new ContourPipeline(0.2, 0.2, 0.2, 0.2);

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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(SPINNER_X, SPINNER_Y), Math.toRadians(SPINNER_ANGLE))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), true)
                .splineTo(new Vector2d(SHIPPING_HUB_X, SHIPPING_HUB_Y), Math.toRadians(SHIPPING_HUB_ANGLE))
                .build();
//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//                .splineTo(new Vector2d(TRANSITION_X, TRANSITION_Y), Math.toRadians(TRANSITION_ANGLE))
//                .splineTo(new Vector2d(WAREHOUSE_X, WAREHOUSE_Y), Math.toRadians(WAREHOUSE_ANGLE))
//                .build();
        TrajectorySequence trajseq3 = drive.trajectorySequenceBuilder((traj2.end()))
                .splineTo(new Vector2d(TRANSITION_X, TRANSITION_Y), Math.toRadians(TRANSITION_ANGLE))
                .splineTo(new Vector2d(WAREHOUSE_X, WAREHOUSE_Y), Math.toRadians(WAREHOUSE_ANGLE))
                .build();
        TrajectorySequence trajseq4 = drive.trajectorySequenceBuilder(trajseq3.end())
                .addTemporalMarker(()->intake.setPower(-1))
                .forward(4)
                .addTemporalMarker(()->intake.setPower(0))
                .build();
        TrajectorySequence trajseq5 = drive.trajectorySequenceBuilder(trajseq4.end())
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(5, trajseq4.end().getY()-10), Math.toRadians(-120))
                .build();
        TrajectorySequence trajseq6 = drive.trajectorySequenceBuilder(trajseq5.end())
                .splineTo(new Vector2d(trajseq4.end().getX(), trajseq4.end().getY()), Math.toRadians(0))
                .build();

        if(magnet.isPressed()) {
            telemetry.addData("Initialized", "Arms in right position");
        } else {
            telemetry.addData("WARNING", "Arms NOT in position!");
        }
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        resetStartTime();
        while (getRuntime() < 3) {
            if (pipeline.error) {
                telemetry.addData("Exception: ", pipeline.debug.getStackTrace());
            }
            double rectangleArea = pipeline.getRectArea();

            //Print out the area of the rectangle that is found.
//            telemetry.addData("Rectangle Area", rectangleArea);
            //Check to see if the rectangle has a large enough area to be a marker.
            if (rectangleArea > minRectangleArea) {
                double cameraWidth = pipeline.getCameraWidth();
                //Then check the location of the rectangle to see which barcode it is in.
                if (pipeline.getRectMidpointX() > rightBarcodeRangeBoundary * cameraWidth) {
                    telemetry.addData("Barcode Position", "Right");
                    tsePos = 3;
                    break;
                } else if (pipeline.getRectMidpointX() < leftBarcodeRangeBoundary * cameraWidth) {
                    telemetry.addData("Barcode Position", "Left");
                    tsePos = 1;
                    break;
                } else {
                    telemetry.addData("Barcode Position", "Center");
                    tsePos = 2;
                    break;
                }
            }
        }
        telemetry.update();

        //drive to carousel and spin.
        drive.followTrajectory(traj1);
        spinner.setPower(1);

        int backArmDegree;
        int foreArmDegree;
        int foreforeArmDumpDegree;
        if (tsePos == 1) {
            backArmDegree = 180;
            foreArmDegree = 210;
            foreforeArmDumpDegree = 180;
            //bottom
        } else if (tsePos == 2) {
            backArmDegree = 180;
            foreArmDegree = 185;
            foreforeArmDumpDegree = 170;
            //mid
        } else {
            backArmDegree = 150;
            foreArmDegree = 129;
            foreforeArmDumpDegree = 200;
            //top
        }
        // start raising arm to drop position
        backArm.setTargetPosition(Common.backArmAngleToEncoder(backArmDegree));
        backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backArm.setVelocity(3000);
        foreArm.setTargetPosition(Common.foreArmAngleToEncoder(foreArmDegree));
        foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreArm.setVelocity(3000);
        foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(90));
        foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreforeArm.setVelocity(500);

        sleep(2500);
        spinner.setPower(0);

        //drive to TSH and drop freight
        drive.followTrajectory(traj2);

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

        // put arm back in
        foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(90));
        foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreforeArm.setVelocity(900);
        backArm.setTargetPosition(Common.backArmAngleToEncoder(180));
        backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backArm.setVelocity(3000);

        //park
        drive.followTrajectorySequenceAsync(trajseq3);
//        drive.followTrajectoryAsync(traj3);
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
        backArm.setTargetPosition(Common.backArmAngleToEncoder(217));
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
        // finish following traj3
        while(drive.isBusy()) {
            drive.update();
        }

        // inch forward to pickup freight
        drive.followTrajectorySequence(trajseq4);
        // go back to shipping hub
        drive.followTrajectorySequence(trajseq5);
        // lift arm and dump
        backArm.setTargetPosition(Common.backArmAngleToEncoder(170));
        backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backArm.setVelocity(4000);
        foreArm.setTargetPosition(Common.foreArmAngleToEncoder(129));
        foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreArm.setVelocity(4000);
        foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(90));
        foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreforeArm.setVelocity(400);
        while(!Common.isInPosition(foreforeArm)) {
            drive.update();
        }
        foreforeArm.setVelocity(0);
        while(!Common.isInPosition(foreArm)) {
            drive.update();
        }
        foreArm.setVelocity(0);
        while(!Common.isInPosition(backArm)) {
            drive.update();
        }
        backArm.setVelocity(0);
        foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(180));
        foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreforeArm.setVelocity(500);
        while (!Common.isInPosition(foreforeArm)) {
            sleep(50);
        }

        // 2nd put arm back in
        foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(90));
        foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreforeArm.setVelocity(900);
        backArm.setTargetPosition(Common.backArmAngleToEncoder(180));
        backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backArm.setVelocity(3000);

        // 2nd park
        drive.followTrajectorySequenceAsync(trajseq6);
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
        backArm.setTargetPosition(Common.backArmAngleToEncoder(217));
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
        // finish following trajseq6
        while(drive.isBusy()) {
            drive.update();
        }
    }

}
