package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    public DcMotor intakeMotor = null;
    public CRServo intakeServo = null;
    private boolean isActive = false;
    private int encoderCount = 0;

    private ElapsedTime clock = new ElapsedTime();
    // Constructor - initializes the intake motor
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        intakeServo = hardwareMap.get(CRServo.class, "IntakeServo");
        //intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void setPower(double power) {
        intakeMotor.setPower(power);
        intakeServo.setPower(power);
    }

    public double getPower(){
        return intakeMotor.getPower();
    }

    public double getRPM(double encoderRes){
        int currentPos = intakeMotor.getCurrentPosition();
        int deltaTicks = currentPos - encoderCount;

        if (deltaTicks != 0) {
            encoderCount = currentPos;
            double time = clock.seconds();
            clock.reset();

            double rotationsMoved = (double) deltaTicks / encoderRes;
            return (rotationsMoved / time)*60;
        }
        else{
            return 0;
        }
    }
}