package org.firstinspires.ftc.teamcode.pedroPathing.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

public class Hub {
    private final List<LynxModule> hubs;
    private final VoltageSensor battery;
    private double volts;
    private long lastVoltsMs;
    private long lastLoopNs;
    private double loopMs;
    private double worstLoopMs;

    public Hub(HardwareMap hw) {
        hubs = hw.getAll(LynxModule.class);
        for (LynxModule m : hubs) m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        battery = hw.voltageSensor.iterator().next();
        volts = battery.getVoltage();
        lastLoopNs = System.nanoTime();
    }

    public void update() {
        for (LynxModule m : hubs) m.clearBulkCache();
        long now = System.nanoTime();
        double ms = (now - lastLoopNs) / 1e6;
        lastLoopNs = now;
        loopMs = loopMs == 0 ? ms : loopMs * 0.9 + ms * 0.1;
        worstLoopMs = Math.max(worstLoopMs * 0.995, ms);
        long nowMs = now / 1000000;
        if (nowMs - lastVoltsMs > 100) {
            lastVoltsMs = nowMs;
            double v = battery.getVoltage();
            if (v > 6) volts = volts * 0.7 + v * 0.3;
        }
    }

    public double volts() {
        return volts;
    }

    public double loopMs() {
        return loopMs;
    }

    public double worstLoopMs() {
        return worstLoopMs;
    }

    public static long nowMs() {
        return System.nanoTime() / 1000000;
    }
}
