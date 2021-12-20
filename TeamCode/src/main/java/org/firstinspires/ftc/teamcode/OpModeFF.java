package org.firstinspires.ftc.teamcode;

//Imports stuff we don't get scary red errors

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//TeleOp :)
@TeleOp
public class OpModeFF extends LinearOpMode {


    private DcMotorEx foreforeArm;
    private DcMotorEx foreArm;
    private DcMotorEx backArm;
    private DcMotorEx intake;

    @Override
    public void runOpMode() {

        foreforeArm = hardwareMap.get(DcMotorEx.class, "foreforearm");
        foreArm = hardwareMap.get(DcMotorEx.class, "forearm");
        backArm = hardwareMap.get(DcMotorEx.class, "backarm");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        //Sets drivetrain motors up
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        boolean BackArmDown;
        boolean ForeArmUp;
        boolean ForeArmDown;
        boolean FFArmUp;
        boolean FFArmDown;
        boolean intakeRun;
        boolean outakeRun;
        //backArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //While the control program is active, the following will run
        while (opModeIsActive()) {
            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
            drive.update();

            BackArmUp = this.gamepad1.y;
            BackArmDown = this.gamepad1.a;
            ForeArmUp = this.gamepad1.dpad_up;
            ForeArmDown = this.gamepad1.dpad_down;
            FFArmUp = this.gamepad1.dpad_left;
            FFArmDown = this.gamepad1.dpad_right;
            intakeRun = this.gamepad1.x;
            outakeRun = this.gamepad1.b;

            //Controls drivetrain
            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            Vector2d input = new Vector2d(
                    y * y * (y > 0 ? -1 : 1),
                    x * x * (x > 0 ? -1 : 1));

            turning = gamepad1.right_stick_x;
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            turning * turning * (turning > 0 ? -1 : 1)));
            //Arm program :)
            boolean backArmUpPress = BackArmUp && !lastBackArmUp;
            boolean backArmUpRelease = !BackArmUp && lastBackArmUp;
            boolean backArmDownPress = BackArmDown && !lastBackArmDown;
            boolean backArmDownRelease = !BackArmDown && lastBackArmDown;

            if (backArmUpPress) {
                //backArm.setVelocity(300);
                backArm.setPower(0.6);
            } else if (backArmDownPress) {
                backArm.setPower(-0.6);
            } else if (backArmUpRelease || backArmDownRelease) {
                backArm.setPower(0);
            }
            lastBackArmUp = BackArmUp;
            lastBackArmDown = BackArmDown;

            boolean foreArmUpPress = ForeArmUp && !lastForeArmUp;
            boolean foreArmUpRelease = !ForeArmUp && lastForeArmUp;
            boolean foreArmDownPress = ForeArmDown && !lastForeArmDown;
            boolean foreArmDownRelease = !ForeArmDown && lastForeArmDown;
            if (foreArmUpPress) {
                //backArm.setVelocity(300);
                foreArm.setPower(0.5);
            } else if (foreArmDownPress) {
                foreArm.setPower(-0.5);
            } else if (foreArmUpRelease || foreArmDownRelease) {
                foreArm.setPower(0);
            }
            lastForeArmUp = ForeArmUp;
            lastForeArmDown = ForeArmDown;

            boolean ffArmUpPress = FFArmUp && !lastFFArmUp;
            boolean ffArmUpRelease = !FFArmUp && lastFFArmUp;
            boolean ffArmDownPress = FFArmDown && !lastFFArmDown;
            boolean ffArmDownRelease = !FFArmDown && lastFFArmDown;
            if (ffArmUpPress) {
                //backArm.setVelocity(30);
                foreforeArm.setPower(0.5);
            } else if (ffArmDownPress) {
                foreforeArm.setPower(-0.5);
            } else if (ffArmUpRelease || ffArmDownRelease) {
                foreforeArm.setPower(0);
            }
            lastFFArmUp = FFArmUp;
            lastFFArmDown = FFArmDown;

            boolean intakeInPress = intakeRun && !lastIntakeIn;
            boolean intakeInRelease = !intakeRun && lastIntakeIn;
            boolean intakeOutPress = outakeRun && !lastIntakeOut;
            boolean intakeOutRelease = !outakeRun && lastIntakeOut;
            if (intakeInPress) {
                //backArm.setVelocity(30);
                intake.setPower(1);
            } else if (intakeOutPress) {
                intake.setPower(-1);
            } else if (intakeInRelease || intakeOutRelease) {
                intake.setPower(0);
            }
            lastIntakeIn = intakeRun;
            lastIntakeOut = outakeRun;

        }
    }
}
