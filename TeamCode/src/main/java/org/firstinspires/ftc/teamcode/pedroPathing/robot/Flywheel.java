package org.firstinspires.ftc.teamcode.pedroPathing.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {
    private final CachedMotor motor;
    private double targetWheelRpm;
    private double tps;
    private double integral;
    private long lastMs;
    private long inBandSinceMs = -1;
    private boolean ready;
    private boolean armed;
    private boolean shot;
    private double lastDip;

    public Flywheel(HardwareMap hw) {
        motor = new CachedMotor(hw, RobotConfig.FLYWHEEL, RobotConfig.FLYWHEEL_REVERSED, false);
    }

    public static double ticksPerWheelRev() {
        return RobotConfig.FLYWHEEL_TICKS_PER_MOTOR_REV / RobotConfig.FLYWHEEL_WHEEL_PER_MOTOR;
    }

    public static double wheelRpmFor(double exitMps) {
        return exitMps / (RobotConfig.EXIT_EFFICIENCY * Math.PI * RobotConfig.FLYWHEEL_WHEEL_DIAMETER_M) * 60;
    }

    public void setExitSpeed(double mps) {
        setWheelRpm(wheelRpmFor(mps));
    }

    public void setWheelRpm(double rpm) {
        if (rpm <= 0) {
            targetWheelRpm = 0;
            return;
        }
        double change = Math.abs(rpm - targetWheelRpm);
        if (change > targetWheelRpm * 0.01) armed = false;
        if (change > targetWheelRpm * RobotConfig.LEAVE_BAND) {
            ready = false;
            inBandSinceMs = -1;
        }
        targetWheelRpm = rpm;
    }

    public void update(double volts, long now) {
        double dt = lastMs == 0 ? 0.02 : Math.max(0.001, (now - lastMs) / 1000.0);
        lastMs = now;
        double raw = motor.velocity();
        tps = tps == 0 ? raw : tps * 0.4 + raw * 0.6;
        if (targetWheelRpm <= 0) {
            motor.setPower(0);
            integral = 0;
            ready = false;
            armed = false;
            inBandSinceMs = -1;
            return;
        }
        double target = targetWheelRpm * ticksPerWheelRev() / 60;
        double err = target - tps;
        integral = clamp(integral + err * dt, -2000, 2000);
        double p = RobotConfig.FLY_KS + RobotConfig.FLY_KV * target + RobotConfig.FLY_KP * err + RobotConfig.FLY_KI * integral;
        p *= RobotConfig.NOMINAL_VOLTAGE / Math.max(8, volts);
        if (err > RobotConfig.BOOST_BELOW * target) p = 1;
        motor.setPower(clamp(p, 0, 1));

        double frac = Math.abs(err) / target;
        if (frac < RobotConfig.READY_BAND) {
            if (inBandSinceMs < 0) inBandSinceMs = now;
            if (now - inBandSinceMs >= RobotConfig.READY_HOLD_MS) {
                ready = true;
                armed = true;
            }
        } else if (frac > RobotConfig.LEAVE_BAND) {
            ready = false;
            inBandSinceMs = -1;
        }
        if (armed && err > RobotConfig.SHOT_DIP * target) {
            shot = true;
            armed = false;
            lastDip = err / target;
        }
    }

    public boolean takeShot() {
        boolean s = shot;
        shot = false;
        return s;
    }

    public void clearShot() {
        shot = false;
    }

    public boolean ready() {
        return ready;
    }

    public double wheelRpm() {
        return tps * 60 / ticksPerWheelRev();
    }

    public double targetRpm() {
        return targetWheelRpm;
    }

    public double power() {
        return motor.power();
    }

    public double lastDip() {
        return lastDip;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
