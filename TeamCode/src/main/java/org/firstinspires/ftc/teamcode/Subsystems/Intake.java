package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    public DcMotor intakeMotor = null;
    private boolean isActive = false;
    private int encoderCount = 0;

    private ElapsedTime clock = new ElapsedTime();
    // Constructor - initializes the intake motor
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");

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


    public void intakeOn(){intakeMotor.setPower(1);}

    public void intakeOff(){intakeMotor.setPower(0);}


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