package org.firstinspires.ftc.teamcode.Game.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
public class Extension {
    private CRServo extension1 = null;
    private CRServo extension2 = null;

    public Extension(HardwareMap hardwareMap){
        extension1 = hardwareMap.get(CRServo.class, "extension1");
        extension2 = hardwareMap.get(CRServo.class, "extension2");
    }

    public void power(double power) {
        extension1.setPower(power);
        extension2.setPower(power);
    }
}
