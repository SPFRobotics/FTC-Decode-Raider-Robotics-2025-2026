package org.firstinspires.ftc.teamcode.Game;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    public DcMotor intakeMotor = null;
    private boolean isActive = false;
    private int encoderCount = 0;

    private ElapsedTime clock = new ElapsedTime();
    // Constructor - initializes the intake motor
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        //intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    // Update method - call this in the main loop with the gamepad
    public void update() {
        // Set motor power based on active state
        if (isActive) {
            intakeMotor.setPower(1.0); // Full power when active
        } else {
            intakeMotor.setPower(0.0); // Off when inactive
        }
    }

    // Getter for active state
    public boolean isActive() {
        return isActive;
    }

    // Manual control methods (optional - for testing or manual override)
    public void activate() {
        isActive = true;
    }

    public void deactivate() {
        isActive = false;
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
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