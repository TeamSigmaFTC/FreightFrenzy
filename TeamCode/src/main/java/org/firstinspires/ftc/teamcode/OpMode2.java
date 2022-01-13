package org.firstinspires.ftc.teamcode;

//Imports stuff we don't get scary red errors
import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Locale;

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

    private double BACK_ARM_ENCODER_ANGLE_RATIO = ((((1.0+(46.0/17.0))) * (1.0+(46.0/17.0))) * 28.0) * 28.0 / 360.0;
    private double BACK_ARM_STARTING_ANGLE = 236;
    // Due to slack, 180 is actually 139, while -180 is actually -221

    private double FORE_ARM_ENCODER_ANGLE_RATIO = ((((1.0+(46.0/17.0))) * (1.0+(46.0/17.0))) * 28.0) * 24.0 / 360.0;
    private double FORE_ARM_STARTING_ANGLE = 23;
    private double FORE_FORE_ARM_ENCODER_ANGLE_RATIO = 2.89 * 3.61 * 3.61 * 28.0 / 360.0;
    private double FORE_FORE_ARM_STARTING_ANGLE = 105;
    public int backArmAngleToEncoder(int degree) {
        return (int) Math.round(BACK_ARM_ENCODER_ANGLE_RATIO*(BACK_ARM_STARTING_ANGLE-degree));
    }

    public int backArmEncoderToAngle(int encoder) {
        return (int) Math.round(BACK_ARM_STARTING_ANGLE - encoder/BACK_ARM_ENCODER_ANGLE_RATIO);
    }

    public int foreArmAngleToEncoder(int degree) {
        return (int) Math.round(FORE_ARM_ENCODER_ANGLE_RATIO * (FORE_ARM_STARTING_ANGLE - degree));
    }

    public int foreArmEncoderToAngle(int encoder) {
        return (int) Math.round(FORE_ARM_STARTING_ANGLE - encoder/FORE_ARM_ENCODER_ANGLE_RATIO);
    }

    public int foreforeArmAngleToEncoder(int degree) {
        return (int) Math.round(-FORE_FORE_ARM_ENCODER_ANGLE_RATIO*(FORE_FORE_ARM_STARTING_ANGLE-degree));
    }

    public int foreforeArmEncoderToAngle(int encoder) {
        return (int) Math.round(FORE_FORE_ARM_STARTING_ANGLE + encoder/FORE_FORE_ARM_ENCODER_ANGLE_RATIO);
    }

    public boolean isInPosition(DcMotorEx motor) {
        return Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) <= motor.getTargetPositionTolerance();
    }

    enum Mode {
        NONE,
        INTAKE_ARM_BACK_MOVE,
        INTAKE_ARM_FORE_MOVE,
        INTAKE_ARM_BACK_MOVE_2,
        INTAKE_ARM_BACK_MOVE_3,
    }

    @Override
    public void runOpMode() {
        Mode currentMode = Mode.NONE;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

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
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backArm.setTargetPositionTolerance(30);
        foreArm.setTargetPositionTolerance(30);

        if(magnet.isPressed()) {
            backArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            foreArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            foreforeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        backArm.setCurrentAlert(4, CurrentUnit.AMPS);
        foreArm.setCurrentAlert(4, CurrentUnit.AMPS);
        foreforeArm.setCurrentAlert(4, CurrentUnit.AMPS);

        //Shows status on driver control station
        telemetry.addData("Status", "Initialized" + (magnet.isPressed()? " with arms encoders reset" : ""));
        telemetry.update();

        //Waits for start
        waitForStart();

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
        boolean lastMid = false;
        boolean lastCap = false;
        boolean lastRed = false;
        boolean lastBlue = false;
        boolean lastShared = false;

        float BackArmDown;
        float ForeArmUp;
        float lastForeArmUp = 0;
        boolean ForeArmDown;
        boolean FFArmUp;
        boolean FFArmDown;
        double intakeRun;
        double outakeRun;
        boolean pickUpPos;
        boolean topPos;
        boolean midPos;
        boolean capPos;
        boolean eStop;
        boolean spinRed;
        boolean spinBlue;
        boolean sharedPos;


        //While the control program is active, the following will run
        while (opModeIsActive()) {
            BackArmUp = -gamepad2.left_stick_y;
            ForeArmUp = this.gamepad2.right_stick_y;
            FFArmDown = this.gamepad2.dpad_up;
            FFArmUp = this.gamepad2.dpad_down;
            intakeRun = this.gamepad1.left_trigger;
            outakeRun = this.gamepad1.right_trigger;
            pickUpPos = this.gamepad2.b; //this.gamepad2.left_bumper;
            topPos = this.gamepad2.x; //this.gamepad2.right_bumper;
            midPos = this.gamepad2.y; //this.gamepad2.b;
            capPos = this.gamepad2.left_bumper;
            eStop  = this.gamepad2.left_stick_button;
            spinRed = this.gamepad1.a;
            spinBlue = this.gamepad1.b;
            sharedPos = this.gamepad2.dpad_right;

            double distance = sensorDistance.getDistance(DistanceUnit.CM);
            telemetry.addData("Color v3 Distance (cm)",
                    String.format(Locale.US, "%.03f", distance));
            Color.RGBToHSV(sensorColor.red(), sensorColor.green(), sensorColor.blue(), hsvValues);
            float hue = hsvValues[0];
            telemetry.addData("Hue", hue);
            if (distance < 6.1 && distance > 4 && hue > 140 && hue < 170) {
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
            Vector2d input = new Vector2d(
                    //Squares power, to allow for more precise robot control
                    y * y * (y > 0 ? -1 : 1),
                    x * x * (x > 0 ? -1 : 1)).rotated(-drive.getPoseEstimate().getHeading() - Math.PI/2);

            turning = gamepad1.right_stick_x;
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            //Squares turning power, to allow for more precise robot control
                            turning * turning * (turning > 0 ? -1 : 1)));
            drive.update();

            //Increases robot efficiency by saving the time it takes for it to constantly reset its
            //power and position when a button is held down
            switch (currentMode) {
                case NONE:
                    //remove power from the arm motors if the arms are in position
                    if(backArm.getMode() == DcMotor.RunMode.RUN_TO_POSITION && isInPosition(backArm)) {
                        backArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        backArm.setVelocity(0);
                    }
                    if(foreArm.getMode() == DcMotor.RunMode.RUN_TO_POSITION && isInPosition(foreArm)) {
                        foreArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        foreArm.setVelocity(0);
                    }
                    if(foreforeArm.getMode() == DcMotor.RunMode.RUN_TO_POSITION && isInPosition(foreforeArm)) {
                        foreforeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        foreforeArm.setVelocity(0);
                    }
                    break;
                case INTAKE_ARM_BACK_MOVE:
                    if (isInPosition(backArm) && isInPosition(foreforeArm)) {
                        backArm.setVelocity(0);
                        foreforeArm.setVelocity(0);
                        currentMode = Mode.INTAKE_ARM_FORE_MOVE;
                        foreArm.setTargetPosition(foreArmAngleToEncoder(310));
                        foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        foreArm.setVelocity(4500);
                    }
                    break;
                case INTAKE_ARM_FORE_MOVE:
                    if (isInPosition(foreArm)) {
                        foreArm.setVelocity(0);
                        currentMode = Mode.INTAKE_ARM_BACK_MOVE_2;
                        foreforeArm.setTargetPosition(foreforeArmAngleToEncoder(-17));
                        foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        foreforeArm.setVelocity(700);
                    }
                    break;
                case INTAKE_ARM_BACK_MOVE_2:
                    if (isInPosition(foreforeArm)){
                        foreforeArm.setVelocity(0);
                        backArm.setTargetPosition(backArmAngleToEncoder(218));
                        backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        backArm.setVelocity(4000);
                        foreArm.setTargetPosition(foreArmAngleToEncoder(325));
                        foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        foreArm.setVelocity(4500);

                        currentMode = Mode.INTAKE_ARM_BACK_MOVE_3;
                    }
                    break;
                case INTAKE_ARM_BACK_MOVE_3:
                    if (isInPosition(backArm) && isInPosition(foreArm)) {
                        backArm.setVelocity(0);
                        foreArm.setVelocity(0);
                        currentMode = Mode.NONE;
                    }
                    break;
            }

            boolean pickUpPress = pickUpPos && !lastPickUp;
            if (pickUpPress) { //backarm to 180 --- foreforearm goes to 270 --- forearm goes to -30 --- once they reach their positions, foreforearm goes to -30 --- once the ff arm gets to its position, backarm move to -210 --- bam done
                //Sets each arm to a specific position, then runs them to their position at a set velocity
                //Moves arm to a position to intake
                currentMode = Mode.INTAKE_ARM_BACK_MOVE;
                backArm.setTargetPosition(backArmAngleToEncoder(180));
                backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backArm.setVelocity(4000);
                foreforeArm.setTargetPosition(foreforeArmAngleToEncoder(90));
                foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreforeArm.setVelocity(700);
            }
            //Saves button state for the next loop.
            lastPickUp = pickUpPress;


            boolean topPosPress = topPos && !lastTop;
            if (topPosPress) {
                //Moves arm to a position to deposit the freight in the top level
                backArm.setTargetPosition(backArmAngleToEncoder(170));
                backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backArm.setVelocity(4000);
                foreArm.setTargetPosition(foreArmAngleToEncoder(129));
                foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreArm.setVelocity(4000);
                foreforeArm.setTargetPosition(foreforeArmAngleToEncoder(90));
                foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreforeArm.setVelocity(400);

            }
            lastTop = topPos;

            boolean midPosPress = midPos && !lastMid;
            if (midPosPress) {
                //Moves arm to a position to deposit the freight in the middle level
                backArm.setTargetPosition(backArmAngleToEncoder(180));
                backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backArm.setVelocity(2000);
                foreArm.setTargetPosition(foreArmAngleToEncoder(145));
                foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreArm.setVelocity(2000);
                foreforeArm.setTargetPosition(foreforeArmAngleToEncoder(90));
                foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreforeArm.setVelocity(400);
            }
            lastMid = midPos;

            boolean capPosPress = capPos && !lastCap;
            if (capPosPress) {
                //Moves arm to a position to cap the team element
                backArm.setTargetPosition(backArmAngleToEncoder(100));
                backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backArm.setVelocity(2000);
                foreArm.setTargetPosition(foreArmAngleToEncoder(150));
                foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreArm.setVelocity(2000);
                foreforeArm.setTargetPosition(foreforeArmAngleToEncoder(90));
                foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreforeArm.setVelocity(400);
            }
            lastCap = capPos;

            boolean sharedPosPress = sharedPos && !lastShared;
            if (sharedPosPress) {
                //Moves arm to a position to cap the team element
                backArm.setTargetPosition(backArmAngleToEncoder(180));
                backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backArm.setVelocity(2000);
                foreArm.setTargetPosition(foreArmAngleToEncoder(270));
                foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreArm.setVelocity(2000);
                foreforeArm.setTargetPosition(foreforeArmAngleToEncoder(90));
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
                intake.setPower(1);
            } else if (intakeOutPress) {
                intake.setPower(-1);
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
                    foreArm.setVelocity(ForeArmUp * 2000);
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
                    foreforeArm.setVelocity(700);
                } else if (ffArmDownPress) {
                    foreforeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    foreforeArm.setVelocity(-700);
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
                currentMode = Mode.NONE;
            }

            //Prints out information on the driver control station screen for puny humans
            telemetry.addData("foerarm power", foreArm.getPower());
            telemetry.addData("estop", eStop);
            telemetry.addData("currentMode", currentMode.toString());
            telemetry.addData("backArm angle", backArmEncoderToAngle(backArm.getCurrentPosition()));
            telemetry.addData("foreArm angle", foreArmEncoderToAngle(foreArm.getCurrentPosition()));
            telemetry.addData("foreforearm angle", foreforeArmEncoderToAngle(foreforeArm.getCurrentPosition()));
            telemetry.addData("foreforearm current", foreforeArm.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();
        }
    }
}
