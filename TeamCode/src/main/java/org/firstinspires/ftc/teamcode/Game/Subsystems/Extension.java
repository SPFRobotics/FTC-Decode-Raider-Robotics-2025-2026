package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Extension {
    @Config
    public static class KickstandValues {
        // These are encoder counts for the motor when the kickstand is up or down.
        public static int kickUpTicks = 0;
        public static int kickDownTicks = 650;
        public static int toleranceTicks = 15;
        public static double kickPower = 0.5;
    }

    private static DcMotor extension1 = null;
    private static DcMotor extension2 = null;
    private static DcMotor kickstand = null;
    private static boolean kickstandIsUp = false;

    public Extension(HardwareMap hardwareMap) {
        setupMotors(hardwareMap);
        setupKickstand(hardwareMap);
    }

    public Extension(HardwareMap hardwareMap, String choice) {
        if ("lift".equals(choice)) {
            setupMotors(hardwareMap);
        } else if ("kickstand".equals(choice)) {
            setupKickstand(hardwareMap);
        }
    }

    private void setupMotors(HardwareMap hardwareMap) {
        extension1 = hardwareMap.get(DcMotor.class, "extension1");
        extension2 = hardwareMap.get(DcMotor.class, "extension2");
    }

    private void setupKickstand(HardwareMap hardwareMap) {
        kickstand = hardwareMap.get(DcMotor.class, "kickstand");
        kickstand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        kickstand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kickstand.setTargetPosition(KickstandValues.kickDownTicks);
        kickstand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        kickstand.setPower(0);
        kickstand.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        if (extension1 == null || extension2 == null) {
            return;
        }
        extension1.setPower(power);
        extension2.setPower(power);
    }

    public void disable(){
        kickstand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        kickstand.setPower(0);
    }

    public void kickStandUp(boolean moveUp) {
        if (kickstand == null) {
            return;
        }

        int upTarget = KickstandValues.kickUpTicks;
        int downTarget = KickstandValues.kickDownTicks;
        int target;
        if (moveUp) {
            target = upTarget;
        } else {
            target = downTarget;
        }

        // Tell the motor to run to the chosen target using the encoder.
        kickstand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        kickstand.setTargetPosition(target);
        kickstand.setPower(KickstandValues.kickPower);
        kickstandIsUp = moveUp;
    }

    public static boolean isKickstandUp() {
        if (kickstand == null) {
            return false;
        }
        int currentTicks = kickstand.getCurrentPosition();
        int goalTicks = KickstandValues.kickUpTicks;
        int difference = Math.abs(currentTicks - goalTicks);
        return difference <= KickstandValues.toleranceTicks;
    }

    public int getKickstandPosition() {
        if (kickstand == null) {
            return 0;
        }
        return kickstand.getCurrentPosition();
    }
}
