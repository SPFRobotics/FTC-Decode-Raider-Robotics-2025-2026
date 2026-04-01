package org.firstinspires.ftc.teamcode.Subsystems.NextFTC;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.OldSubsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.OldSubsystems.Turret;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

import java.util.Collections;
import java.util.Set;

@Config
public class NextOuttake implements Subsystem {

    public static final NextOuttake INSTANCE = new NextOuttake();

    public static double kP = 0.00983;
    public static double kI = 0.0;
    public static double kD = 0.000001;

    public static double kV = 0.000380;
    public static double kS = 0.135;

    public static double farRPM = 3300;
    public static double closeRPM = 2600;
    public static double gearRatio = 18.0 / 16.0;
    public static double rpmTolerance = 100;

    private static final int ENCODER_TICKS = 28;

    private final MotorEx motor = new MotorEx("OuttakeMotor").reversed();

    private ControlSystem controlSystem;

    private KickerSpindex kicker = null;
    private Turret turret = null;
    private boolean isFarLocation = true;
    private boolean launched = false;
    private int kickerCycleCount = 0;
    private final ElapsedTime interval = new ElapsedTime();

    private NextOuttake() {}

    public void setKicker(KickerSpindex kicker) {
        if (kicker == null) {
            throw new RuntimeException("Kicker must not be null!");
        }
        this.kicker = kicker;
    }

    public void setTurret(Turret turret) {
        this.turret = turret;
    }

    @Override
    public void initialize() {
        controlSystem = ControlSystem.builder()
                .velPid(kP, kI, kD)
                .basicFF(kV, 0.0, kS)
                .build();
        controlSystem.setGoal(new KineticState(0, 0, 0));
        launched = false;
        kickerCycleCount = 0;
        interval.reset();
    }

    @Override
    public void periodic() {
        motor.setPower(controlSystem.calculate(motor.getState()));
    }


    public void setRPM(double rpm) {
        controlSystem.setGoal(new KineticState(0, rpmToTps(rpm), 0));
    }

    public void stop() {
        controlSystem.setGoal(new KineticState(0, 0, 0));
        controlSystem.reset();
    }

    public double getRPM() {
        return tpsToRpm(motor.getVelocity());
    }

    public double getPower() {
        return motor.getPower();
    }


    public Command spinUpFar() {
        return new RunToVelocity(controlSystem, rpmToTps(farRPM))
                .requires(this)
                .named("OuttakeSpinUpFar");
    }

    public Command spinUpClose() {
        return new RunToVelocity(controlSystem, rpmToTps(closeRPM))
                .requires(this)
                .named("OuttakeSpinUpClose");
    }

    public Command spinUp(double rpm) {
        return new RunToVelocity(controlSystem, rpmToTps(rpm))
                .requires(this)
                .named("OuttakeSpinUp(" + rpm + ")");
    }

    public Command stopCommand() {
        return new InstantCommand("OuttakeStop", this::stop)
                .requires(this);
    }


    public void switchLocation() {
        isFarLocation = !isFarLocation;
    }

    public boolean isFarLocation() {
        return isFarLocation;
    }

    public String getLocationName() {
        return isFarLocation ? "FAR" : "SHORT";
    }

    public double distanceToRPM(double distance) {
        double x = distance;
        double RPS = (Math.sqrt((9.8 * x * x)
                / ((0.075) * ((5.062 * x) - 0.454025)))) / 0.603;
        return RPS * 60;
    }


    public void enableKickerCycle(boolean x, double RPM) {
        requireKicker();
        double time = interval.seconds();
        if (x) {
            if (time >= 1 && time < 2 && getRPM() >= RPM - 500) {
                kicker.up();
                launched = true;
            } else if (time >= 2) {
                kicker.down();
                if (launched) {
                    kickerCycleCount++;
                }
                launched = false;
                interval.reset();
            }
        } else {
            kicker.up();
            interval.reset();
        }
    }

    public void enableSpindexKickerCycle(boolean x, double RPM) {
        requireKicker();
        double time = interval.seconds();
        if (x) {
            if (!launched && Math.abs(RPM - getRPM()) <= rpmTolerance) {
                kicker.up();
                launched = true;
                interval.reset();
            } else if (launched && time >= 0.1) {
                kicker.down();
                kickerCycleCount++;
                launched = false;
                interval.reset();
            }
        } else {
            kicker.up();
            interval.reset();
        }
    }

    public int getKickerCycleCount() {
        return kickerCycleCount;
    }

    public void resetKickerCycle() {
        requireKicker();
        kickerCycleCount = 0;
        launched = false;
        interval.reset();
        kicker.down();
    }

    public double getCurrentCycleTime() {
        return interval.seconds();
    }


    public void shortAuto() {
        double RPM = 2700;
        setRPM(RPM);
        enableKickerCycle(true, RPM);
    }


    private double rpmToTps(double rpm) {
        return (rpm / 60.0) * ENCODER_TICKS * gearRatio;
    }

    private double tpsToRpm(double tps) {
        return (tps * 60.0) / ENCODER_TICKS / gearRatio;
    }

    private void requireKicker() {
        if (kicker == null) {
            throw new RuntimeException("Kicker not set! Call setKicker() first.");
        }
    }


    @Override
    public Command getDefaultCommand() {
        return null;
    }

    @Override
    public Set<Subsystem> getSubsystems() {
        return Collections.singleton(this);
    }
}
