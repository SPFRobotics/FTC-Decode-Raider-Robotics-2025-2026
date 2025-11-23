package org.firstinspires.ftc.teamcode.Game.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
public class Extension {
    private static DcMotor extension1 = null;
    private static DcMotor extension2 = null;

    public Extension(HardwareMap hardwareMap){
        extension1 = hardwareMap.get(DcMotor.class, "extension1");
        extension2 = hardwareMap.get(DcMotor.class, "extension2");
    }

    public static void setPower(double power) {
        extension1.setPower(power);
        extension2.setPower(power);
    }
}
