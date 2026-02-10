package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {

    public DcMotorEx intakeMotor = null;
    private boolean isActive = false;
    private int encoderCount = 0;
    private boolean isReversed = false;

    private ElapsedTime clock = new ElapsedTime();
    private ElapsedTime jamTimer = new ElapsedTime();
    // Constructor - initializes the intake motor
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        intakeMotor.setCurrentAlert(8.5,  CurrentUnit.AMPS);
        /*
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         */
    }


    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public double getPower(){
        return intakeMotor.getPower();
    }

    public double getAmps(){
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }

    public void intakeOn(){
        intakeMotor.setPower(1);

    }

    public void intakeOn(boolean jamDetection){
        if (intakeMotor.isOverCurrent() && jamDetection && !isReversed){
            jamTimer.reset();
            isReversed = true;
        }

        if (jamTimer.seconds() > 1.0){
            intakeMotor.setPower(1);
            isReversed = false;
        }
        else{
            intakeMotor.setPower(-1);
        }
    }

    public void intakeOff(){
        intakeMotor.setPower(0);
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