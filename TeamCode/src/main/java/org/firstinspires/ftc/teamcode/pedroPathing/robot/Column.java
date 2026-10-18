package org.firstinspires.ftc.teamcode.pedroPathing.robot;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Column {
    private final CachedMotor motor;
    private final DigitalChannel in, top;
    private final StallGuard stall = new StallGuard();
    private boolean inBroken, topBroken, inEdge;
    private long noProgressSinceMs = -1;
    private boolean stuck;

    public Column(HardwareMap hw) {
        motor = new CachedMotor(hw, RobotConfig.COLUMN, RobotConfig.COLUMN_REVERSED, true);
        in = hw.get(DigitalChannel.class, RobotConfig.COLUMN_IN);
        top = hw.get(DigitalChannel.class, RobotConfig.COLUMN_TOP);
        in.setMode(DigitalChannel.Mode.INPUT);
        top.setMode(DigitalChannel.Mode.INPUT);
    }

    public void read() {
        boolean wasIn = inBroken;
        inBroken = broken(in);
        topBroken = broken(top);
        inEdge = inBroken && !wasIn;
    }

    private static boolean broken(DigitalChannel c) {
        return c.getState() != RobotConfig.BEAM_BROKEN_IS_LOW;
    }

    public void update(boolean feeding, int held, long now) {
        double p = 0;
        if (feeding) {
            p = RobotConfig.COLUMN_FEED_POWER;
            noProgressSinceMs = -1;
        } else if (held > 0 && !topBroken && !stuck) {
            p = RobotConfig.COLUMN_INDEX_POWER;
            if (noProgressSinceMs < 0) noProgressSinceMs = now;
            if (now - noProgressSinceMs > RobotConfig.COLUMN_STUCK_MS) stuck = true;
        } else if (held > 0 && inBroken && !stuck) {
            p = RobotConfig.COLUMN_INDEX_POWER * 0.5;
        } else {
            noProgressSinceMs = -1;
        }
        if (topBroken) stuck = false;
        double amps = p == 0 ? 0 : motor.amps(now, 100);
        motor.setPower(stall.filter(p, amps, now));
    }

    public void clearStuck() {
        stuck = false;
        noProgressSinceMs = -1;
    }

    public boolean entered() {
        return inEdge;
    }

    public boolean staged() {
        return topBroken;
    }

    public boolean entryBlocked() {
        return inBroken;
    }

    public boolean stuck() {
        return stuck;
    }

    public int jams() {
        return stall.jams();
    }
}
