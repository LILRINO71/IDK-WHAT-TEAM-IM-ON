package org.firstinspires.ftc.teamcode.pedroPathing.robot;

public class HeadingLock {
    private double lastErr = Double.NaN;
    private long lastMs;
    private double dErr;

    public double turn(double targetRad, double headingRad, long now) {
        double err = Field.wrap(targetRad - headingRad);
        double dt = lastMs == 0 ? 0.02 : Math.max(0.001, (now - lastMs) / 1000.0);
        if (!Double.isNaN(lastErr)) dErr = dErr * 0.6 + ((err - lastErr) / dt) * 0.4;
        lastErr = err;
        lastMs = now;
        double out = RobotConfig.HEADING_KP * err + RobotConfig.HEADING_KD * dErr;
        if (Math.abs(Math.toDegrees(err)) > RobotConfig.AIM_TOLERANCE_DEG * 0.5) out += Math.signum(err) * RobotConfig.HEADING_KS;
        return Math.max(-0.8, Math.min(0.8, out));
    }

    public void reset() {
        lastErr = Double.NaN;
        lastMs = 0;
        dErr = 0;
    }
}
