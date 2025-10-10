package org.firstinspires.ftc.teamcode.Game;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {

    public DcMotor outtakeMotor = null;
    private boolean isActive = false;
    private int encoderCount = 0;
    private int direction = 1; // 1 for forward, -1 for backward

    private ElapsedTime clock = new ElapsedTime();
    // Constructor - initializes the intake motor
    public Outtake(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotor.class, "OuttakeMotor");
        //intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    // Update method - call this in the main loop with the gamepad
    public void update() {
        // Set motor power based on active state and direction
        if (isActive) {
            outtakeMotor.setPower(-1.0 * direction); // Full power when active, respects direction
        } else {
            outtakeMotor.setPower(0.0); // Off when inactive
        }
    }
    
    // Flip the direction of the outtake
    public void flipDirection() {
        direction *= -1;
    }
    
    // Get current direction (1 for forward, -1 for backward)
    public int getDirection() {
        return direction;
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
        outtakeMotor.setPower(power);
    }

    public double getRPM(double encoderRes){
        int currentPos = outtakeMotor.getCurrentPosition();
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