package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class DTtest extends OpMode {

    DcMotor motor;

    @Override
    public void init() {

        motor = hardwareMap.get(DcMotor.class, "backLeftDrive");
    }

    @Override
    public void loop() {

        motor.setPower(1);

    }
}
