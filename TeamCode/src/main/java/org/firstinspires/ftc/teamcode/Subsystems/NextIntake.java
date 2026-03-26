package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

import java.util.Collections;
import java.util.Set;

public class NextIntake implements Subsystem {

    public static final NextIntake INSTANCE = new NextIntake();

    private final MotorEx motor = new MotorEx("IntakeMotor");

    private final ElapsedTime jamTimer = new ElapsedTime();
    private final ElapsedTime waitOnStart = new ElapsedTime();
    private boolean isReversed = false;

    private int lastEncoderPos = 0;
    private final ElapsedTime rpmTimer = new ElapsedTime();

    // Commands
    public final Command intakeOn = new SetPower(motor, 1.0).requires(this).named("IntakeOn");
    public final Command intakeOff = new SetPower(motor, 0.0).requires(this).named("IntakeOff");
    public final Command intakeReverse = new SetPower(motor, -1.0).requires(this).named("IntakeReverse");

    private NextIntake() {
        motor.getMotor().setCurrentAlert(7.7, CurrentUnit.AMPS);
    }

    @Override
    public void initialize() {
        motor.setPower(0);
        isReversed = false;
        waitOnStart.reset();
    }

    @Override
    public void periodic() {
        // Jam detection logic runs every loop when intake is active
        if (motor.getPower() > 0 && motor.getMotor().isOverCurrent() && !isReversed) {
            jamTimer.reset();
            isReversed = true;
        }

        if (isReversed) {
            if (jamTimer.seconds() > 1.0) {
                motor.setPower(1.0);
                isReversed = false;
            } else {
                motor.setPower(-1.0);
            }
        }
    }

    // Direct control methods (for use outside NextFTCOpMode)
    public void setPower(double power) {
        motor.setPower(power);
    }

    public double getMotorPower() {
        return motor.getPower();
    }

    public void turnOn() {
        motor.setPower(1.0);
    }

    public void turnOff() {
        motor.setPower(0.0);
    }

    // Utility methods
    public double getAmps() {
        return motor.getMotor().getCurrent(CurrentUnit.AMPS);
    }

    public double getCurrent() {
        return motor.getMotor().getCurrent(CurrentUnit.MILLIAMPS);
    }

    public double getRPM(double encoderRes) {
        int currentPos = (int) motor.getCurrentPosition();
        int deltaTicks = currentPos - lastEncoderPos;

        if (deltaTicks != 0) {
            lastEncoderPos = currentPos;
            double time = rpmTimer.seconds();
            rpmTimer.reset();
            double rotations = (double) deltaTicks / encoderRes;
            return (rotations / time) * 60;
        }
        return 0;
    }

    // Subsystem interface defaults
    @Override
    public Command getDefaultCommand() {
        return null;
    }

    @Override
    public Set<Subsystem> getSubsystems() {
        return Collections.singleton(this);
    }
}
