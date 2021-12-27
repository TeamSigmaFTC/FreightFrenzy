package org.firstinspires.ftc.teamcode;

//Imports stuff we don't get scary red errors
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class OpMode2 extends LinearOpMode {


    private DcMotorEx foreforeArm;
    private DcMotorEx foreArm;
    private DcMotorEx backArm;
    private DcMotorEx intake;
    private CRServo spinner;

    private double BACK_ARM_ENCODER_ANGLE_RATIO = ((((1.0+(46.0/17.0))) * (1.0+(46.0/17.0))) * 28.0) * 28.0 / 360.0;
    private double BACK_ARM_STARTING_ANGLE = 238;
    // Due to slack, 180 is actually 139, while -180 is actually -221
    private double FORE_ARM_ENCODER_ANGLE_RATIO = ((((1.0+(46.0/17.0))) * (1.0+(46.0/17.0))) * 28.0) * 24.0 / 360.0;
    private double FORE_ARM_STARTING_ANGLE = 24;
    private double FORE_FORE_ARM_ENCODER_ANGLE_RATIO = 2.89 * 3.61 * 3.61 * 28.0 / 360.0;
    private double FORE_FORE_ARM_STARTING_ANGLE = 105;
    public int backArmAngleToEncoder(int degree) {
        return (int) Math.round(BACK_ARM_ENCODER_ANGLE_RATIO*(BACK_ARM_STARTING_ANGLE-degree));
    }

    public int backArmEncoderToAngle(int encoder) {
        return (int) Math.round(BACK_ARM_STARTING_ANGLE - encoder/BACK_ARM_ENCODER_ANGLE_RATIO);
    }

//    private int FORE_ARM_SLACK_AT_90 = 46;
    public int foreArmAngleToEncoder(int degree) {
//        if (degree >= 90 + FORE_ARM_SLACK_AT_90) {
//            degree -= FORE_ARM_SLACK_AT_90;
//        }
//        if (degree >= 270 && degree <= 360) {
//            degree -= FORE_ARM_SLACK_AT_90 * (degree -270)/90;
//        }
        return (int) Math.round(FORE_ARM_ENCODER_ANGLE_RATIO * (FORE_ARM_STARTING_ANGLE - degree));
    }

    public int foreArmEncoderToAngle(int encoder) {
        return (int) Math.round(FORE_ARM_STARTING_ANGLE - encoder/FORE_ARM_ENCODER_ANGLE_RATIO);
//        if(degree >= 90){
//            degree += FORE_ARM_SLACK_AT_90;
//        }
//        if (degree >= 270 && degree <= 360) {
//            degree -= FORE_ARM_SLACK_AT_90 * (degree -270)/90;
//        }
//        return degree;
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

        //Sets drivetrain motors up
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //backArm.setTargetPositionTolerance(25);
        foreArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //foreArm.setTargetPositionTolerance(25);
        foreforeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Shows status on driver control station
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //Waits for start
        waitForStart();

        //Sets some values
        double x;
        double y;
        double turning;
        boolean BackArmUp;
        boolean lastBackArmUp = false;
        boolean lastForeArmUp = false;
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

        boolean BackArmDown;
        boolean ForeArmUp;
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
            BackArmUp = this.gamepad1.y;
            BackArmDown = this.gamepad1.a;
            ForeArmUp = this.gamepad1.dpad_up;
            ForeArmDown = this.gamepad1.dpad_down;
            FFArmUp = this.gamepad1.dpad_up;
            FFArmDown = this.gamepad1.dpad_down;
            intakeRun = this.gamepad1.left_trigger;
            outakeRun = this.gamepad1.right_trigger;
            pickUpPos = this.gamepad1.left_bumper; //this.gamepad2.left_bumper;
            topPos = this.gamepad1.y; //this.gamepad2.right_bumper;
            midPos = this.gamepad1.x; //this.gamepad2.b;
            capPos = this.gamepad1.right_stick_button;
            eStop  = this.gamepad1.left_stick_button;
            spinRed = this.gamepad1.b;
            spinBlue = this.gamepad1.right_bumper;
            sharedPos = this.gamepad1.dpad_right;

            //Controls drivetrain
            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            Vector2d input = new Vector2d(
                    //Squares power, to allow for more precise robot control
                    y * y * (y > 0 ? -1 : 1),
                    x * x * (x > 0 ? -1 : 1));

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
                case INTAKE_ARM_BACK_MOVE:
                    if (isInPosition(backArm)) {
                        backArm.setVelocity(0);
                        currentMode = Mode.INTAKE_ARM_FORE_MOVE;
                        foreArm.setTargetPosition(foreArmAngleToEncoder(325));
                        foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        foreArm.setVelocity(2000);
                        foreforeArm.setTargetPosition(foreforeArmAngleToEncoder(-17));
                        foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        foreforeArm.setVelocity(300);
                    }
                    break;
                case INTAKE_ARM_FORE_MOVE:
                    if (isInPosition(foreforeArm) && isInPosition(foreArm)) {
                        foreforeArm.setVelocity(0);
                        foreArm.setVelocity(0);
                        currentMode = Mode.INTAKE_ARM_BACK_MOVE_2;
                        backArm.setTargetPosition(backArmAngleToEncoder(218));
                        backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        backArm.setVelocity(1000);
                    }
                    break;
                case INTAKE_ARM_BACK_MOVE_2:
                    if (isInPosition(backArm)){
                        backArm.setVelocity(0);
                        currentMode = Mode.NONE;
                    }
                    break;
//                case INTAKE_ARM_BACK_MOVE_2:
//                    if (isInPosition(backArm)) {
//                        backArm.setVelocity(0);
//                        currentMode = Mode.INTAKE_ARM_FORE_MOVE_2;
//                        foreArm.setTargetPosition(foreArmAngleToEncoder(-20));
//                        foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        foreArm.setVelocity(1000);
//                    }
//                    break;
//                case INTAKE_ARM_FORE_MOVE_2:
//                    foreArm.setVelocity(0);
//                    currentMode = Mode.NONE;
//                    break;
            }
            boolean pickUpPress = pickUpPos && !lastPickUp;
            if (pickUpPress) { //backarm to 180 --- foreforearm goes to 270 --- forearm goes to -30 --- once they reach their positions, foreforearm goes to -30 --- once the ff arm gets to its position, backarm move to -210 --- bam done
                //Sets each arm to a specific position, then runs them to their position at a set velocity
                //Moves arm to a position to intake
                currentMode = Mode.INTAKE_ARM_BACK_MOVE;
                backArm.setTargetPosition(backArmAngleToEncoder(180));
                backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backArm.setVelocity(1000);
            }
            //Saves button state for the next loop.
            lastPickUp = pickUpPress;


            boolean topPosPress = topPos && !lastTop;
            if (topPosPress) {
                //Moves arm to a position to deposit the freight in the top level
                backArm.setTargetPosition(backArmAngleToEncoder(180));
                backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backArm.setVelocity(1000);
                foreArm.setTargetPosition(foreArmAngleToEncoder(129));
                foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreArm.setVelocity(1000);
                foreforeArm.setTargetPosition(foreforeArmAngleToEncoder(90));
                foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreforeArm.setVelocity(200);

            }
            lastTop = topPos;

            boolean midPosPress = midPos && !lastMid;
            if (midPosPress) {
                //Moves arm to a position to deposit the freight in the middle level
                backArm.setTargetPosition(backArmAngleToEncoder(180));
                backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backArm.setVelocity(1000);
                foreArm.setTargetPosition(foreArmAngleToEncoder(145));
                foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreArm.setVelocity(1000);
                foreforeArm.setTargetPosition(foreforeArmAngleToEncoder(90));
                foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreforeArm.setVelocity(200);
            }
            lastMid = midPos;

            boolean capPosPress = capPos && !lastCap;
            if (capPosPress) {
                //Moves arm to a position to cap the team element
                backArm.setTargetPosition(backArmAngleToEncoder(100));
                backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backArm.setVelocity(1000);
                foreArm.setTargetPosition(foreArmAngleToEncoder(150));
                foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreArm.setVelocity(1000);
                foreforeArm.setTargetPosition(foreforeArmAngleToEncoder(90));
                foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreforeArm.setVelocity(200);
            }
            lastCap = capPos;

            boolean sharedPosPress = sharedPos && !lastShared;
            if (sharedPosPress) {
                //Moves arm to a position to cap the team element
                backArm.setTargetPosition(backArmAngleToEncoder(180));
                backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backArm.setVelocity(1000);
                foreArm.setTargetPosition(foreArmAngleToEncoder(270));
                foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreArm.setVelocity(1000);
                foreforeArm.setTargetPosition(foreforeArmAngleToEncoder(90));
                foreforeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                foreforeArm.setVelocity(200);
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

            boolean ffArmUpPress = FFArmUp && !lastFFArmUp;
            boolean ffArmDownPress = FFArmDown && !lastFFArmDown;
            if (ffArmUpPress) {
                foreforeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                foreforeArm.setVelocity(200);
            } else if (ffArmDownPress) {
                foreforeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                foreforeArm.setVelocity(-200);
            } else if (eStop) {
                foreforeArm.setVelocity(0);
            }
            lastFFArmUp = FFArmUp;
            lastFFArmDown = FFArmDown;



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

            if (eStop) {
                //if emergency stop
                backArm.setVelocity(0);
                foreArm.setVelocity(0);
                foreforeArm.setVelocity(0);
                currentMode = Mode.NONE;
            }

            //Test Program

            //Checks if button is pressed, and if the button was pressed in the last loop
//                boolean backArmUpPress = BackArmUp && !lastBackArmUp;
//                boolean backArmDownPress = BackArmDown && !lastBackArmDown;
//
//                //Back arm control program
//                if (backArmUpPress) {
//                    //When backArmUp is pressed, set power to 60%
//                    backArm.setTargetPosition(backArmAngleToEncoder(90));
//                    backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    backArm.setVelocity(1000);
//                } else if (backArmDownPress) {
//                    //When backArmDown is pressed, set power to negative 60%
//                    backArm.setTargetPosition(backArmAngleToEncoder(180));
//                    backArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    backArm.setVelocity(1000);
//                } else if (eStop) {
//                    //If either of the two controls are released, set power to 0%
//                    backArm.setVelocity(0);
//                }
//                //Saves button state for the next loop.
//                lastBackArmUp = BackArmUp;
//                lastBackArmDown = BackArmDown;
//
//                //Same program as the one above, just a different arm
//                boolean foreArmUpPress = ForeArmUp && !lastForeArmUp;
//                boolean foreArmDownPress = ForeArmDown && !lastForeArmDown;
//                if (foreArmUpPress) {
//                    foreArm.setTargetPosition(foreArmAngleToEncoder(90));
//                    foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    foreArm.setVelocity(1000);
//                } else if (foreArmDownPress) {
//                    foreArm.setTargetPosition(foreArmAngleToEncoder(180));
//                    foreArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    foreArm.setVelocity(1000);
//                } else if (eStop) {
//                    foreArm.setVelocity(0);
//                }
//                lastForeArmUp = ForeArmUp;
//                lastForeArmDown = ForeArmDown;
//
//




            //Prints out information on the driver control station screen for puny humans
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
