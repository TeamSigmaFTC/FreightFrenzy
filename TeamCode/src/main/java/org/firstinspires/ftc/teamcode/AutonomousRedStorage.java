package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Collection;

@Config
@Autonomous
public class AutonomousRedStorage extends LinearOpMode {

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

    protected Pose2d startPose = new Pose2d(-36, -63, Math.toRadians(90));
    public static double SPINNER_X = -60;
    public static double SPINNER_Y = -56;
    public static double SPINNER_ANGLE = 270;
    public static double SHIPPING_HUB_X = -36;
    public static double SHIPPING_HUB_Y = -48;
    public static double SHIPPING_HUB_ANGLE = 45;
    public static double STORAGE_X = -58;
    public static double STORAGE_Y = -35;
    public static double STORAGE_ANGLE = 0;

    // Green Range                                      Y      Cr     Cb
//    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 0.0);
//    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0);
    // red range
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 170.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 150.0);
    // blue range
//    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 150);
//    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 160.0, 255.0);


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

        // Only if you are using ftcdashboard
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
//        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(SPINNER_X, SPINNER_Y), Math.toRadians(SPINNER_ANGLE))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), true)
                .splineTo(new Vector2d(SHIPPING_HUB_X, SHIPPING_HUB_Y), Math.toRadians(SHIPPING_HUB_ANGLE))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(-48,-54 ), Math.toRadians(90))
                .splineTo(new Vector2d(STORAGE_X, STORAGE_Y), Math.toRadians(STORAGE_ANGLE))
                .build();


        waitForStart();
        if (isStopRequested()) return;

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

        //drive to carousel and spin.
        drive.followTrajectory(traj1);
        spinner.setPower(-1);
        sleep(3000);
        spinner.setPower(0);

        //drive to TSH and drop freight
        drive.followTrajectory(traj2);

        int backArmDegree;
        int foreArmDegree;
        int foreforeArmDumpDegree;
        if (tsePos == 1) {
            backArmDegree = 180;
            foreArmDegree = 200;
            foreforeArmDumpDegree = 180;
            //bottom
        }else if (tsePos == 2){
            backArmDegree = 180;
            foreArmDegree = 170;
            foreforeArmDumpDegree = 170;
            //mid
        }else {
            backArmDegree = 160;
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
        drive.followTrajectory(traj3);

    }

}
