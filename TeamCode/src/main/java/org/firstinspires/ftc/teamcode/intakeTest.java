package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
public class intakeTest extends LinearOpMode{
    private DcMotorEx intake;

    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.y) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
        }
    }
}
