package org.firstinspires.ftc.teamcode.Game;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake {

    public DcMotor intakeMotor = null;
    private boolean isActive = false;
    private boolean lastSquareState = false;

    // Constructor - initializes the intake motor
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
    }

    // Update method - call this in the main loop with the gamepad
    public void update(Gamepad gamepad) {
        // Toggle logic: detect button press (not held)
        if (gamepad.square && !lastSquareState) {
            isActive = !isActive; // Toggle on/off
        }
        lastSquareState = gamepad.square;

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
}