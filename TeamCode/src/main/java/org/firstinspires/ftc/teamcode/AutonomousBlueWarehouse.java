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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous
public class AutonomousBlueWarehouse extends LinearOpMode {

    private DcMotorEx foreforeArm;
    private DcMotorEx foreArm;
    private DcMotorEx backArm;

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

    public static double SHIPPING_HUB_X = 12;
    public static double SHIPPING_HUB_Y = 49;
    public static double SHIPPING_HUB_ANGLE = 45;

    public static double TRANSITION1_X = -9;
    public static double TRANSITION1_Y = 60;
    public static double TRANSITION1_ANGLE = 180;

    public static double TRANSITION2_X = 16;
    public static double TRANSITION2_Y = 65;
    public static double TRANSITION2_ANGLE = 0;

    public static double WAREHOUSE_X = 42;
    public static double WAREHOUSE_Y = 65;
    public static double WAREHOUSE_ANGLE = 0;

    // Green Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0);



    @Override
    public void runOpMode() throws InterruptedException {

        spinner = hardwareMap.get(CRServo.class, "spinner");
        foreforeArm = hardwareMap.get(DcMotorEx.class, "foreforearm");
        foreArm = hardwareMap.get(DcMotorEx.class, "forearm");
        backArm = hardwareMap.get(DcMotorEx.class, "backarm");
        backArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //backArm.setTargetPositionTolerance(25);
        foreArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //foreArm.setTargetPositionTolerance(25);
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

        // Only if you are using ftcdashboard
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
//        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(SHIPPING_HUB_X, SHIPPING_HUB_Y), Math.toRadians(SHIPPING_HUB_ANGLE))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), true)
                .splineTo(new Vector2d(TRANSITION1_X, TRANSITION1_Y), Math.toRadians(TRANSITION1_ANGLE))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(TRANSITION2_X, TRANSITION2_Y), Math.toRadians(TRANSITION2_ANGLE))
                .splineTo(new Vector2d(WAREHOUSE_X, WAREHOUSE_Y), Math.toRadians(WAREHOUSE_ANGLE))
                .build();

        waitForStart();
        if (isStopRequested()) return;
        //getRuntime() < 2
        while (true) {
            if (pipeline.error) {
                telemetry.addData("Exception: ", pipeline.debug.getStackTrace());
            }
            double rectangleArea = pipeline.getRectArea();

            //Print out the area of the rectangle that is found.
            telemetry.addData("Rectangle Area", rectangleArea);
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

        //drive to TSH and drop freight
        drive.followTrajectory(traj1);
        int backArmDegree;
        int foreArmDegree;
        int foreforeArmDumpDegree;
        if (tsePos == 1) {
            backArmDegree = 170;
            foreArmDegree = 210;
            foreforeArmDumpDegree = 180;
            //bottom
        }else if (tsePos == 2){
            backArmDegree = 180;
            foreArmDegree = 169;
            foreforeArmDumpDegree = 180;
            //mid
        }else {
            backArmDegree = 150;
            foreArmDegree = 129;
            foreforeArmDumpDegree = 218;
            //top
        }
        backArm.setTargetPosition(Common.backArmAngleToEncoder(backArmDegree));
        backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backArm.setVelocity(1000);
        foreArm.setTargetPosition(Common.foreArmAngleToEncoder(foreArmDegree));
        foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreArm.setVelocity(1000);
        foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(90));
        foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreforeArm.setVelocity(200);
        while (!Common.isInPosition(backArm)){
            sleep(50);
        }
        backArm.setVelocity(0);
        while (!Common.isInPosition(foreArm)){
            sleep(50);
        }
        foreArm.setVelocity(0);
        while (!Common.isInPosition(foreforeArm)){
            sleep(50);
        }
        foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder( foreforeArmDumpDegree));
        foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreforeArm.setVelocity(200);
        while (!Common.isInPosition(foreforeArm)){
            sleep(50);
        }
        //put arm back in
        foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder((int) Common.FORE_FORE_ARM_STARTING_ANGLE));
        foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreforeArm.setVelocity(300);

        while (!Common.isInPosition(foreforeArm)){
            sleep(50);
        }
        foreforeArm.setVelocity(0);
        foreArm.setTargetPosition(Common.foreArmAngleToEncoder((int) Common.FORE_ARM_STARTING_ANGLE));
        foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foreArm.setVelocity(2000);

        backArm.setTargetPosition(Common.backArmAngleToEncoder((int) Common.BACK_ARM_STARTING_ANGLE));
        backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backArm.setVelocity(1000);
        while (!Common.isInPosition(backArm)){
            sleep(50);
        }
        backArm.setVelocity(0);
        while (!Common.isInPosition(foreArm)){
            sleep(50);
        }
        foreArm.setVelocity(0);

        //park
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

    }

}
