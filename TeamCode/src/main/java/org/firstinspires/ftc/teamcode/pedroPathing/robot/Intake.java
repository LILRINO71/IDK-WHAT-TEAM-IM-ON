package org.firstinspires.ftc.teamcode.pedroPathing.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public enum Mode { OFF, IN, EJECT }

    public class Side {
        public final String name;
        final CachedMotor motor;
        final BallSensor sensor;
        final StallGuard stall = new StallGuard();
        Side other;
        String state = "idle";
        Ball waiting = Ball.NONE;
        long untilMs;
        long claimUntilMs;
        long goneSinceMs = -1;
        int seenArrivals;

        Side(HardwareMap hw, String name, String motorName, boolean reversed, String sensorName) {
            this.name = name;
            motor = new CachedMotor(hw, motorName, reversed, false);
            sensor = new BallSensor(hw, sensorName);
        }

        double step(Mode mode, Inventory inv, Alliance alliance, long now) {
            if (mode == Mode.OFF) {
                state = "idle";
                seenArrivals = -1;
                return 0;
            }
            if (mode == Mode.EJECT) {
                state = "idle";
                seenArrivals = -1;
                return RobotConfig.INTAKE_EJECT_POWER;
            }
            sensor.update();
            Ball b = sensor.ball();
            boolean fresh = b != Ball.NONE && sensor.arrivals() != seenArrivals;
            switch (state) {
                case "eject":
                    if (now < untilMs) return RobotConfig.INTAKE_EJECT_POWER;
                    state = "idle";
                    return 0;
                case "wait":
                    if (other.claimUntilMs > now) return 0;
                    return accept(waiting, inv, now);
                case "feed":
                    if (fresh) return decide(b, inv, alliance, now);
                    if (sensor.present()) {
                        goneSinceMs = -1;
                    } else {
                        if (goneSinceMs < 0) goneSinceMs = now;
                        if (now - goneSinceMs > 40) state = "idle";
                    }
                    return RobotConfig.INTAKE_POWER;
                default:
                    if (fresh) return decide(b, inv, alliance, now);
                    if (inv.full()) return 0;
                    return sensor.present() ? RobotConfig.INTAKE_SLOW_POWER : RobotConfig.INTAKE_POWER;
            }
        }

        private double decide(Ball b, Inventory inv, Alliance alliance, long now) {
            seenArrivals = sensor.arrivals();
            if (!b.ours(alliance)) {
                rejectedOpponent = true;
                return eject(now);
            }
            if (inv.full()) {
                rejectedFull = true;
                return eject(now);
            }
            if (other.claimUntilMs > now) {
                waiting = b;
                state = "wait";
                return 0;
            }
            return accept(b, inv, now);
        }

        private double eject(long now) {
            state = "eject";
            untilMs = now + (long) RobotConfig.EJECT_MS;
            return RobotConfig.INTAKE_EJECT_POWER;
        }

        private double accept(Ball b, Inventory inv, long now) {
            if (inv.full()) {
                rejectedFull = true;
                return eject(now);
            }
            inv.accepted(b, now);
            claimUntilMs = now + (long) RobotConfig.MERGE_HOLD_MS;
            state = "feed";
            goneSinceMs = -1;
            waiting = Ball.NONE;
            return RobotConfig.INTAKE_POWER;
        }

        public String state() {
            return state;
        }

        public BallSensor sensor() {
            return sensor;
        }

        public int jams() {
            return stall.jams();
        }
    }

    public final Side front, back;
    private boolean rejectedOpponent, rejectedFull;

    public Intake(HardwareMap hw) {
        front = new Side(hw, "front", RobotConfig.FRONT_INTAKE, RobotConfig.FRONT_INTAKE_REVERSED, RobotConfig.FRONT_COLOR);
        back = new Side(hw, "back", RobotConfig.BACK_INTAKE, RobotConfig.BACK_INTAKE_REVERSED, RobotConfig.BACK_COLOR);
        front.other = back;
        back.other = front;
    }

    public void update(Mode mode, Inventory inv, Alliance alliance, long now) {
        run(front, mode, inv, alliance, now);
        run(back, mode, inv, alliance, now);
    }

    private void run(Side s, Mode mode, Inventory inv, Alliance alliance, long now) {
        double p = s.step(mode, inv, alliance, now);
        double amps = p == 0 ? 0 : s.motor.amps(now, 100);
        s.motor.setPower(s.stall.filter(p, amps, now));
    }

    public boolean takeRejectedOpponent() {
        boolean r = rejectedOpponent;
        rejectedOpponent = false;
        return r;
    }

    public boolean takeRejectedFull() {
        boolean r = rejectedFull;
        rejectedFull = false;
        return r;
    }
}
