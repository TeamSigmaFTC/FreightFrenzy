package org.firstinspires.ftc.teamcode;

//Imports stuff we don't get scary red errors
import android.graphics.Color;

import com.ThermalEquilibrium.homeostasis.Filters.Estimators.Estimator;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.LowPassEstimator;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Locale;
import java.util.function.DoubleSupplier;

@TeleOp
public class OpMode2 extends LinearOpMode {


    private DcMotorEx foreforeArm;
    private DcMotorEx foreArm;
    private DcMotorEx backArm;
    private DcMotorEx intake;
    private CRServo spinner;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    TouchSensor magnet;
    DigitalChannel redLED1;
    DigitalChannel greenLED1;
    RevBlinkinLedDriver blinkinLedDriver;

    enum ArmMode {
        NONE,
        INTAKE_ARM_BACK_MOVE,
        INTAKE_ARM_FORE_MOVE,
        INTAKE_ARM_BACK_MOVE_2,
        INTAKE_ARM_BACK_MOVE_3,
        TOP_TRAY_POS_MOVE,
        TOP_TRAY_POS_MOVE_2,
        TOP_POS_FFA,
    }

    enum DriveMode {
        NONE,
        AUTO,
    }

    @Override
    public void runOpMode() {
        ArmMode currentArmMode = ArmMode.NONE;
        DriveMode currentDriveMode = DriveMode.NONE;

//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        foreforeArm = hardwareMap.get(DcMotorEx.class, "foreforearm");
        foreArm = hardwareMap.get(DcMotorEx.class, "forearm");
        backArm = hardwareMap.get(DcMotorEx.class, "backarm");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        spinner = hardwareMap.get(CRServo.class, "spinner");
        sensorColor = hardwareMap.get(ColorSensor.class, "color");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color");
        magnet = hardwareMap.get(TouchSensor.class, "magnet");
        redLED1 = hardwareMap.get(DigitalChannel.class, "red1");
        greenLED1 = hardwareMap.get(DigitalChannel.class, "green1");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        //hsvValues is an array that will hold the hue, saturation, and value information
        float hsvValues[] = {0F, 0F, 0F};

        //values is a reference to the hsvValues array
        final float values[] = hsvValues;

        //Sets drivetrain motors up
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        drive.addT265(hardwareMap);
//        drive.useT265(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backArm.setTargetPositionTolerance(10);
        foreArm.setTargetPositionTolerance(15);

//        if(magnet.isPressed()) {
            backArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            foreArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            foreforeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }

        backArm.setCurrentAlert(5, CurrentUnit.AMPS);
        foreArm.setCurrentAlert(5, CurrentUnit.AMPS);
        foreforeArm.setCurrentAlert(5, CurrentUnit.AMPS);

        double Q = 0.3;
        double R = 3;
        int N = 5;
        DoubleSupplier intakeDistanceSensor = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return sensorDistance.getDistance(DistanceUnit.CM);
            }
        };

//        Estimator intakeDiistanceEstimator = new KalmanEstimator(intakeDistanceSensor,Q,R,N);

        Estimator intakeDiistanceEstimator = new LowPassEstimator(intakeDistanceSensor, 0.9);
        //Shows status on driver control station
        telemetry.addData("Status", "Initialized" + (magnet.isPressed()? " with arms encoders reset" : ""));
        telemetry.addData("pose", Storage.currentPose);
        telemetry.update();

        //Waits for start
        waitForStart();

        drive.setPoseEstimate(Storage.currentPose);
        if(drive.t265 != null){
            drive.t265.setPoseEstimate(Storage.currentPose);
        } else  if (drive.getLocalizer() instanceof LocalizerT265){
            LocalizerT265 loc = (LocalizerT265) drive.getLocalizer();
            if(loc.defaultLocalizer != null) {
                loc.defaultLocalizer.setPoseEstimate(Storage.currentPose);
            }
        }

        // change LED mode from input to output
        redLED1.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED1.setMode(DigitalChannel.Mode.OUTPUT);

        //Sets some values
        double x;
        double y;
        double turning;
        float BackArmUp;
        float lastBackArmUp = 0;
        boolean lastForeArmDown = false;
        boolean lastBackArmDown = false;
        boolean lastFFArmUp = false;
        boolean lastFFArmDown = false;
        boolean lastIntakeIn = false;
        boolean lastIntakeOut = false;

        boolean lastPickUp = false;
        boolean lastTop = false;
        boolean lastLow = false;
        boolean lastCap = false;
        boolean lastRed = false;
        boolean lastBlue = false;
        boolean lastShared = false;

        float ForeArmUp;
        float lastForeArmUp = 0;
        boolean fastSpeedButton;
        boolean slowSpeedButton;
        boolean FFArmUp;
        boolean FFArmDown;
        double intakeRun;
        double outakeRun;
        boolean pickUpPos;
        boolean topPos;
        boolean lowPos;
        boolean capPos;
        boolean eStop;
        boolean spinRed;
        boolean spinBlue;
        boolean sharedPos;

        double speedMultipler = 0.7;


        //While the control program is active, the following will run
        while (opModeIsActive()) {
            BackArmUp = -gamepad2.left_stick_y;
            ForeArmUp = this.gamepad2.right_stick_y;
            FFArmDown = this.gamepad2.dpad_up;
            FFArmUp = this.gamepad2.dpad_down;
            intakeRun = this.gamepad1.right_trigger;
            outakeRun = this.gamepad1.left_trigger;
            fastSpeedButton = this.gamepad1.dpad_up;
            slowSpeedButton = this.gamepad1.dpad_down;
            pickUpPos = this.gamepad2.b; //this.gamepad2.left_bumper;
            topPos = this.gamepad2.x; //this.gamepad2.right_bumper;
            lowPos = this.gamepad2.y; //this.gamepad2.b;
            capPos = this.gamepad2.right_bumper;
            eStop  = this.gamepad1.left_bumper || this.gamepad2.left_bumper;
            spinRed = this.gamepad1.a;
            spinBlue = this.gamepad1.b;
            sharedPos = this.gamepad2.dpad_right;

//            double distance = sensorDistance.getDistance(DistanceUnit.CM);
            double distance = intakeDiistanceEstimator.update();
            telemetry.addData("Color v3 Distance (cm)",
                    String.format(Locale.US, "%.03f", distance));
            Color.RGBToHSV(sensorColor.red(), sensorColor.green(), sensorColor.blue(), hsvValues);
            float hue = hsvValues[0];
            telemetry.addData("Hue", hue);
            if (distance < 5.7 && distance > 4 && hue > 140 && hue < 170) {
                // if the sensor is looking through the hole on the basket, show green.
                redLED1.setState(false);
                greenLED1.setState(true);
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else if (hue > 140 && hue < 148 && distance < 0.85) {
                // color green and very close to the sensor, show red color on the led
                redLED1.setState(true);
                greenLED1.setState(false);
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (hue < 140 && hue > 75 && distance < 4) {
                //show amber if the detected color is yellow-ish
                redLED1.setState(false);
                greenLED1.setState(false);
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else if (hue > 154 && hue < 165 && distance < 3) {
                //show amber if the detected color is white-ish
                redLED1.setState(false);
                greenLED1.setState(false);
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            } else {
                // turn led off
                redLED1.setState(true);
                greenLED1.setState(true);
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }

            //Controls drivetrain
            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            switch (currentDriveMode) {
                case NONE:
                    if(slowSpeedButton) {
                        speedMultipler = 0.7;
                    } else if (fastSpeedButton) {
                        speedMultipler = 1;
                    }
                    Pose2d poseEstimate = drive.getPoseEstimate();
                    Vector2d input = new Vector2d(
                            //Squares power, to allow for more precise robot control
                            y * y * (y > 0 ? -1 : 1),
                            x * x * (x > 0 ? -1 : 1));//.rotated(-poseEstimate.getHeading() - Math.PI/2);

                    turning = gamepad1.right_stick_x;
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    input.getX() * speedMultipler,
                                    input.getY() * speedMultipler,
                                    //Squares turning power, to allow for more precise robot control
                                    turning * turning * (turning > 0 ? -1 : 1) * .7));
//                    if (gamepad1.y) {
//                        currentDriveMode = DriveMode.AUTO;
//                        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        if(poseEstimate.getX() > 24) {
//                            //inside warehouse, drive out to scoring location
//                            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(poseEstimate)
//                                    .lineToLinearHeading(new Pose2d(42, 65, 0))
//                                    .setTangent(Math.toRadians(180)) // follow the path backwards
//                                    .splineTo(new Vector2d(5, 53), Math.toRadians(-120))
//                                    .build());
//                        } else {
//                            //outside warehouse, drive back to warehouse
//                            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(poseEstimate)
//                                    .lineToLinearHeading(new Pose2d(6, 65, Math.toRadians(0)))
//                                    .lineTo(new Vector2d(40, 65))
//                                    .build());
//                        }
//                    }
                    break;
                case AUTO:
                    if (gamepad1.x) {
                        drive.cancelFollowing();
                        currentDriveMode = DriveMode.NONE;
                        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }
                    if (!drive.isBusy()) {
                        currentDriveMode = DriveMode.NONE;
                        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }
                    break;
            }

            drive.update();

            //Increases robot efficiency by saving the time it takes for it to constantly reset its
            //power and position when a button is held down
            switch (currentArmMode) {
                case NONE:
                    //remove power from the arm motors if the arms are in position
                    if(backArm.getMode() == DcMotor.RunMode.RUN_TO_POSITION && Common.isInPosition(backArm)) {
                        backArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        backArm.setVelocity(0);
                    }
                    if(foreArm.getMode() == DcMotor.RunMode.RUN_TO_POSITION && Common.isInPosition(foreArm)) {
                        foreArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        foreArm.setVelocity(0);
                    }
                    if(foreforeArm.getMode() == DcMotor.RunMode.RUN_TO_POSITION && Common.isInPosition(foreforeArm)) {
                        foreforeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        foreforeArm.setVelocity(0);
                    }
                    break;
                case INTAKE_ARM_BACK_MOVE:
                    if (Common.isInPosition(backArm) && Common.isInPosition(foreforeArm)) {
                        backArm.setVelocity(0);
                        foreforeArm.setVelocity(0);
                        currentArmMode = ArmMode.INTAKE_ARM_FORE_MOVE;
                        foreArm.setTargetPosition(Common.foreArmAngleToEncoder(355));
                        foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        foreArm.setVelocity(4500);
                    }
                    break;
                case INTAKE_ARM_FORE_MOVE:
                    if (Common.isInPosition(foreArm)) {
                        foreArm.setVelocity(0);
                        currentArmMode = ArmMode.INTAKE_ARM_BACK_MOVE_2;
                        foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(205));
                        foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        foreforeArm.setVelocity(1000);
                    }
                    break;
                case INTAKE_ARM_BACK_MOVE_2:
                    if (Common.isInPosition(foreforeArm)){
                        foreforeArm.setVelocity(0);
                        backArm.setTargetPosition(Common.backArmAngleToEncoder(225));
                        backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        backArm.setVelocity(4000);
//                        foreArm.setTargetPosition(Common.foreArmAngleToEncoder(325));
//                        foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        foreArm.setVelocity(4500);

//                        currentArmMode = ArmMode.INTAKE_ARM_BACK_MOVE_3;
                        currentArmMode = ArmMode.NONE;

                    }
                    break;
                case TOP_POS_FFA: {
                    if (Common.isInPosition(backArm)){
                        foreArm.setTargetPosition(Common.foreArmAngleToEncoder(180));
                        foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        foreArm.setVelocity(3000);
                        foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(103));
                        foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        foreforeArm.setVelocity(500);
                        currentArmMode = ArmMode.NONE;
                    }
                    }
//                case INTAKE_ARM_BACK_MOVE_3:
//                    if (Common.isInPosition(backArm) && Common.isInPosition(foreArm)) {
//                        backArm.setVelocity(0);
//                        foreArm.setVelocity(0);
//                        currentArmMode = ArmMode.NONE;
//                    }
//                    break;
//                case TOP_TRAY_POS_MOVE:
//                    if (Common.isInPosition(backArm)) {
//                        backArm.setVelocity(0);
//                        currentArmMode = ArmMode.TOP_TRAY_POS_MOVE_2;
////                        foreArm.setTargetPosition(Common.foreArmAngleToEncoder(129));
////                        foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        foreArm.setVelocity(2000);
////                        foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(90));
////                        foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        foreforeArm.setVelocity(400);
//                    }
//                    break;
//                case TOP_TRAY_POS_MOVE_2:
//                    if(Common.isInPosition(foreArm) && Common.isInPosition(foreforeArm)) {
//                        foreArm.setVelocity(0);
//                        foreforeArm.setVelocity(0);
//                        currentArmMode = ArmMode.NONE;
//                    }
//                    break;
            }

            boolean pickUpPress = pickUpPos && !lastPickUp;
            if (pickUpPress) { //backarm to 180 --- foreforearm goes to 270 --- forearm goes to -30 --- once they reach their positions, foreforearm goes to -30 --- once the ff arm gets to its position, backarm move to -210 --- bam done
                //Sets each arm to a specific position, then runs them to their position at a set velocity
                //Moves arm to a position to intake
                currentArmMode = ArmMode.INTAKE_ARM_BACK_MOVE;
                backArm.setTargetPosition(Common.backArmAngleToEncoder(180));
                backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backArm.setVelocity(4000);
                foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(30));
                foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreforeArm.setVelocity(700);
            }
            //Saves button state for the next loop.
            lastPickUp = pickUpPress;


            boolean topPosPress = topPos && !lastTop;
            if (topPosPress) {
                //Moves arm to a position to deposit the freight in the top level
                currentArmMode = ArmMode.TOP_POS_FFA;
                backArm.setTargetPosition(Common.backArmAngleToEncoder(180));
                backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backArm.setVelocity(3000);
//                currentArmMode = ArmMode.TOP_TRAY_POS_MOVE;
            }
            lastTop = topPos;

            boolean lowPosPress = lowPos && !lastLow;
            if (lowPosPress) {
                //Moves arm to a position to deposit the freight in the low level
                backArm.setTargetPosition(Common.backArmAngleToEncoder(180));
                backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backArm.setVelocity(2000);
                foreArm.setTargetPosition(Common.foreArmAngleToEncoder(280));
                foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreArm.setVelocity(800);
                foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(90));
                foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreforeArm.setVelocity(400);
            }
            lastLow = lowPos;

            boolean capPosPress = capPos && !lastCap;
            if (capPosPress) {
                //Moves arm to a position to cap the team element
//                backArm.setTargetPosition(Common.backArmAngleToEncoder(100));
//                backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                backArm.setVelocity(2000);
//                foreArm.setTargetPosition(Common.foreArmAngleToEncoder(150));
//                foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                foreArm.setVelocity(2000);
//                foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(90));
//                foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                foreforeArm.setVelocity(400);
            }
            lastCap = capPos;

            boolean sharedPosPress = sharedPos && !lastShared;
            if (sharedPosPress) {
                //Moves arm to a position to cap the team element
                backArm.setTargetPosition(Common.backArmAngleToEncoder(180));
                backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backArm.setVelocity(2000);
                foreArm.setTargetPosition(Common.foreArmAngleToEncoder(270));
                foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreArm.setVelocity(2000);
                foreforeArm.setTargetPosition(Common.foreforeArmAngleToEncoder(90));
                foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreforeArm.setVelocity(400);
            }
            lastShared = sharedPos;

            boolean spinnerRed = spinRed && !lastRed;
            boolean redRelease = !spinRed && lastRed;
            boolean spinnerBlue = spinBlue && !lastBlue;
            boolean blueRelease = !spinBlue && lastBlue;
            if (spinnerRed) {
                spinner.setPower(1);
            } else if (spinnerBlue) {
                spinner.setPower(-1);
            } else if (redRelease || blueRelease) {
                spinner.setPower(0);
            }
            lastRed = spinRed;
            lastBlue = spinBlue;


            boolean intakeInPress = intakeRun > 0.1 && !lastIntakeIn;
            boolean intakeInRelease = intakeRun < 0.1 && lastIntakeIn;
            boolean intakeOutPress = outakeRun > 0.1 && !lastIntakeOut;
            boolean intakeOutRelease = outakeRun < 0.1 && lastIntakeOut;
            if (intakeInPress) {
                intake.setPower(-1);
            } else if (intakeOutPress) {
                intake.setPower(1);
            } else if (intakeInRelease || intakeOutRelease) {
                intake.setPower(0);
            }
            lastIntakeIn = intakeRun > 0.1;
            lastIntakeOut = outakeRun > 0.1;

            if (backArm.isOverCurrent()) {
                backArm.setVelocity(0);
            } else {
                boolean backArmMoved = Math.abs(BackArmUp - lastBackArmUp) > 0.05;
                if(backArmMoved) {
                    backArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    backArm.setVelocity(BackArmUp * 2000);
                } else if (Math.abs(BackArmUp) < 0.01 && Math.abs(lastBackArmUp) > 0.01) {
                    backArm.setVelocity(0);
                }
                lastBackArmUp = BackArmUp;
            }
            if (foreArm.isOverCurrent()) {
                foreArm.setVelocity(0);
            } else {
                boolean foreArmMoved = Math.abs(ForeArmUp - lastForeArmUp) > 0.05;
                if(foreArmMoved) {
                    foreArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    foreArm.setPower(ForeArmUp);
//                    foreArm.setVelocity(ForeArmUp * 2000);
                } else if (Math.abs(ForeArmUp) < 0.01 && Math.abs(lastForeArmUp) > 0.01) {
                    foreArm.setVelocity(0);
                }
                lastForeArmUp = ForeArmUp;
            }
            if (foreforeArm.isOverCurrent()) {
                foreforeArm.setVelocity(0);
            } else {
                boolean ffArmUpPress = FFArmUp && !lastFFArmUp;
                boolean ffArmUpRelease = !FFArmUp && lastFFArmUp;
                boolean ffArmDownPress = FFArmDown && !lastFFArmDown;
                boolean ffArmDownRelease = !FFArmDown && lastFFArmDown;

                if (ffArmUpPress) {
                    foreforeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    foreforeArm.setVelocity(1000);
                } else if (ffArmDownPress) {
                    foreforeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    foreforeArm.setVelocity(-1000);
                } else if (ffArmUpRelease || ffArmDownRelease) {
                    foreforeArm.setVelocity(0);
                }
                lastFFArmUp = FFArmUp;
                lastFFArmDown = FFArmDown;
            }

            if (eStop) {
                //if emergency stop
                backArm.setVelocity(0);
                foreArm.setVelocity(0);
                foreforeArm.setVelocity(0);
                currentArmMode = ArmMode.NONE;
            }

            //Prints out information on the driver control station screen for puny humans
            if(drive.t265!=null) {
                telemetry.addData("pose", drive.getPoseEstimate());
                telemetry.addData("pose T265", drive.t265.getPoseEstimate());
            }else if(drive.getLocalizer() instanceof LocalizerT265){
                telemetry.addData("pose t265", drive.getPoseEstimate());
                telemetry.addData("pose", ((LocalizerT265)drive.getLocalizer()).defaultLocalizer.getPoseEstimate());
            }
//            telemetry.addData("estop", eStop);
            telemetry.addData("currentMode", currentArmMode.toString());
            telemetry.addData("backArm angle", Common.backArmEncoderToAngle(backArm.getCurrentPosition()));
            telemetry.addData("foreArm angle", Common.foreArmEncoderToAngle(foreArm.getCurrentPosition()));
            telemetry.addData("foreforearm angle", Common.foreforeArmEncoderToAngle(foreforeArm.getCurrentPosition()));
            telemetry.addData("foreforearm current", foreforeArm.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();
        }
    }
}
