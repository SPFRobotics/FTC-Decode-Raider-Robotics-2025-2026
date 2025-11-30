package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Extension {
    @Config
    public static class KickstandValues {
        public static double kickUp = 0.75;
        public static double kickDown = 0.15;
    }

    private static DcMotor extension1 = null;
    private static DcMotor extension2 = null;
    private static Servo kickstand = null;
    private static boolean kickstandIsUp = false;

    public Extension(HardwareMap hardwareMap) {
        setupMotors(hardwareMap);
        setupKickstand(hardwareMap);
    }

    public Extension(HardwareMap hardwareMap, String choice) {
        if ("motor".equals(choice)) {
            setupMotors(hardwareMap);
        } else if ("servo".equals(choice)) {
            setupKickstand(hardwareMap);
        }
    }

    private void setupMotors(HardwareMap hardwareMap) {
        extension1 = hardwareMap.get(DcMotor.class, "extension1");
        extension2 = hardwareMap.get(DcMotor.class, "extension2");
    }

    private void setupKickstand(HardwareMap hardwareMap) {
        kickstand = hardwareMap.get(Servo.class, "kickstand");
    }

    public static void setPower(double power) {
        if (extension1 == null || extension2 == null) {
            return;
        }
        extension1.setPower(power);
        extension2.setPower(power);
    }

    public static void kickStandUp(boolean moveUp) {
        if (kickstand == null) {
            return;
        }

        double targetPosition;
        if (moveUp) {
            targetPosition = KickstandValues.kickUp;
        } else {
            targetPosition = KickstandValues.kickDown;
        }
        kickstand.setPosition(targetPosition);
        kickstandIsUp = moveUp;
    }

    public static boolean isKickstandUp() {
        return kickstandIsUp;
    }
}
