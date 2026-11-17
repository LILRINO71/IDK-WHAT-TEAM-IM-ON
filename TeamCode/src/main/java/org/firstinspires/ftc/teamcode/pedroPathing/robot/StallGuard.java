package org.firstinspires.ftc.teamcode.pedroPathing.robot;

public class StallGuard {
    private enum State { RUN, BACK_OFF, COOLDOWN }

    private State state = State.RUN;
    private long overSinceMs = -1;
    private long changedMs;
    private int jams;

    public double filter(double commanded, double amps, long nowMs) {
        switch (state) {
            case BACK_OFF:
                if (nowMs - changedMs > RobotConfig.UNJAM_MS) {
                    state = State.COOLDOWN;
                    changedMs = nowMs;
                }
                return commanded == 0 ? 0 : RobotConfig.UNJAM_POWER * Math.signum(commanded);
            case COOLDOWN:
                if (nowMs - changedMs > RobotConfig.UNJAM_COOLDOWN_MS) state = State.RUN;
                return commanded;
            default:
                if (Math.abs(commanded) > 0.3 && amps > RobotConfig.STALL_AMPS) {
                    if (overSinceMs < 0) overSinceMs = nowMs;
                    if (nowMs - overSinceMs > RobotConfig.STALL_MS) {
                        state = State.BACK_OFF;
                        changedMs = nowMs;
                        overSinceMs = -1;
                        jams++;
                    }
                } else {
                    overSinceMs = -1;
                }
                return commanded;
        }
    }

    public boolean unjamming() {
        return state == State.BACK_OFF;
    }

    public int jams() {
        return jams;
    }
}
