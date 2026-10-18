package org.firstinspires.ftc.teamcode.pedroPathing.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    public enum State { IDLE, FEEDING, CLOSING }

    public final Flywheel flywheel;
    private final Servo pitch, stopper;
    private State state = State.IDLE;
    private long stateMs;
    private double pitchDeg = Double.NaN;
    private long pitchSettledAtMs;
    private boolean fired;
    private boolean leftTop;
    private int dryFires;
    public double speedTrim = 1.0;
    public double pitchTrimDeg = 0;

    public Shooter(HardwareMap hw) {
        flywheel = new Flywheel(hw);
        pitch = hw.get(Servo.class, RobotConfig.PITCH_SERVO);
        stopper = hw.get(Servo.class, RobotConfig.STOPPER_SERVO);
        stopper.setPosition(RobotConfig.STOPPER_CLOSED);
    }

    public void aimAt(double pitchDegrees, double exitMps, long now) {
        setPitch(pitchDegrees + pitchTrimDeg, now);
        flywheel.setExitSpeed(exitMps * speedTrim);
    }

    public void idleAt(double exitMps, long now) {
        flywheel.setExitSpeed(exitMps * RobotConfig.IDLE_FRACTION);
    }

    public void stop() {
        flywheel.setWheelRpm(0);
    }

    public void setPitch(double deg, long now) {
        deg = Math.max(RobotConfig.PITCH_MIN_DEG, Math.min(RobotConfig.PITCH_MAX_DEG, deg));
        if (!Double.isNaN(pitchDeg) && Math.abs(deg - pitchDeg) < 0.25) return;
        double move = Double.isNaN(pitchDeg) ? 30 : Math.abs(deg - pitchDeg);
        pitchDeg = deg;
        double t = (deg - RobotConfig.PITCH_MIN_DEG) / (RobotConfig.PITCH_MAX_DEG - RobotConfig.PITCH_MIN_DEG);
        pitch.setPosition(RobotConfig.PITCH_SERVO_AT_MIN + t * (RobotConfig.PITCH_SERVO_AT_MAX - RobotConfig.PITCH_SERVO_AT_MIN));
        pitchSettledAtMs = Math.max(pitchSettledAtMs, now + (long) (move * RobotConfig.PITCH_SETTLE_MS_PER_DEG));
    }

    public void update(boolean wantFire, boolean aimed, boolean staged, double volts, long now) {
        fired = false;
        flywheel.update(volts, now);
        switch (state) {
            case IDLE:
                flywheel.clearShot();
                if (wantFire && staged && aimed && flywheel.ready() && now >= pitchSettledAtMs) {
                    stopper.setPosition(RobotConfig.STOPPER_OPEN);
                    state = State.FEEDING;
                    stateMs = now;
                    leftTop = false;
                }
                break;
            case FEEDING:
                if (!staged) leftTop = true;
                boolean dip = flywheel.takeShot();
                if (dip || (leftTop && now - stateMs > 120)) {
                    fired = true;
                    close(now);
                } else if (now - stateMs > RobotConfig.FEED_TIMEOUT_MS) {
                    dryFires++;
                    close(now);
                }
                break;
            case CLOSING:
                flywheel.clearShot();
                if (now - stateMs > RobotConfig.STOPPER_MOVE_MS) state = State.IDLE;
                break;
        }
    }

    private void close(long now) {
        stopper.setPosition(RobotConfig.STOPPER_CLOSED);
        state = State.CLOSING;
        stateMs = now;
    }

    public boolean feeding() {
        return state == State.FEEDING;
    }

    public boolean pushing() {
        return state == State.FEEDING && !leftTop;
    }

    public boolean firedThisLoop() {
        return fired;
    }

    public boolean pitchSettled(long now) {
        return now >= pitchSettledAtMs;
    }

    public double pitchDeg() {
        return pitchDeg;
    }

    public State state() {
        return state;
    }

    public int dryFires() {
        return dryFires;
    }
}
