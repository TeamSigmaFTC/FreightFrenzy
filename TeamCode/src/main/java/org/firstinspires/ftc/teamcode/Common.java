package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Common {
    private static double BACK_ARM_ENCODER_ANGLE_RATIO = ((((1.0+(46.0/17.0))) * (1.0+(46.0/17.0))) * 28.0) * 28.0 / 360.0;
    public static double BACK_ARM_STARTING_ANGLE = 236;
    // Due to slack, 180 is actually 139, while -180 is actually -221

    private static double FORE_ARM_ENCODER_ANGLE_RATIO = ((((1.0+(46.0/17.0))) * (1.0+(46.0/17.0))) * 28.0) * 24.0 / 360.0;
    public static double FORE_ARM_STARTING_ANGLE = 23;
    private static double FORE_FORE_ARM_ENCODER_ANGLE_RATIO = 2.89 * 3.61 * 3.61 * 28.0 / 360.0;
    public static double FORE_FORE_ARM_STARTING_ANGLE = 105;
    public static int backArmAngleToEncoder(int degree) {
        return (int) Math.round(BACK_ARM_ENCODER_ANGLE_RATIO*(BACK_ARM_STARTING_ANGLE-degree));
    }

    public static int backArmEncoderToAngle(int encoder) {
        return (int) Math.round(BACK_ARM_STARTING_ANGLE - encoder/BACK_ARM_ENCODER_ANGLE_RATIO);
    }

    public static int foreArmAngleToEncoder(int degree) {
        return (int) Math.round(FORE_ARM_ENCODER_ANGLE_RATIO * (FORE_ARM_STARTING_ANGLE - degree));
    }

    public static int foreArmEncoderToAngle(int encoder) {
        return (int) Math.round(FORE_ARM_STARTING_ANGLE - encoder/FORE_ARM_ENCODER_ANGLE_RATIO);
    }

    public static int foreforeArmAngleToEncoder(int degree) {
        return (int) Math.round(-FORE_FORE_ARM_ENCODER_ANGLE_RATIO*(FORE_FORE_ARM_STARTING_ANGLE-degree));
    }

    public static int foreforeArmEncoderToAngle(int encoder) {
        return (int) Math.round(FORE_FORE_ARM_STARTING_ANGLE + encoder/FORE_FORE_ARM_ENCODER_ANGLE_RATIO);
    }

    public static boolean isInPosition(DcMotorEx motor) {
        return Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) <= motor.getTargetPositionTolerance();
    }

}
