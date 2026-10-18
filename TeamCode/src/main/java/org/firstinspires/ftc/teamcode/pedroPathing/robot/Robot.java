package org.firstinspires.ftc.teamcode.pedroPathing.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Robot {
    public final Hub hub;
    public final Follower follower;
    public final Intake intake;
    public final Column column;
    public final Shooter shooter;
    public final HiveTracker hive;
    public final JammingWheels jam;
    public final Inventory inv = new Inventory();
    public final Aim aim = new Aim();
    public final HeadingLock lock = new HeadingLock();
    public Alliance alliance;

    public Intake.Mode intakeMode = Intake.Mode.OFF;
    public boolean wantAim, wantFire, spinWhenHolding = true;
    public double gramsInUpCell;

    private int seenFlips;
    private boolean tipSure;
    private boolean tipSureEdge;
    private long now;

    public Robot(HardwareMap hw, Alliance alliance, Pose start) {
        this.alliance = alliance;
        hub = new Hub(hw);
        follower = Constants.createFollower(hw);
        follower.setStartingPose(start);
        intake = new Intake(hw);
        column = new Column(hw);
        shooter = new Shooter(hw);
        hive = new HiveTracker(hw);
        jam = new JammingWheels(hw);
        follower.update();
    }

    public void update() {
        now = Hub.nowMs();
        hub.update();
        follower.update();
        hive.update(now);
        inv.update(now);

        int flips = hive.flips(alliance);
        if (flips != seenFlips) {
            seenFlips = flips;
            gramsInUpCell = 0;
        }

        column.read();
        if (column.entered()) inv.enteredColumn();

        aim.solve(follower.getPose(), follower.getVelocity(), alliance, hive.upSide(alliance), inv.next());
        if (wantAim || wantFire) shooter.aimAt(aim.pitchDeg, aim.speedMps, now);
        else if (spinWhenHolding && inv.held() > 0) shooter.idleAt(aim.speedMps, now);
        else shooter.stop();

        shooter.update(wantFire, aim.aimed(), column.staged(), hub.volts(), now);
        if (shooter.firedThisLoop()) {
            Ball b = inv.fired();
            gramsInUpCell += b.grams();
        }
        boolean sure = gramsInUpCell >= RobotConfig.TIP_SURE_G;
        tipSureEdge = sure && !tipSure;
        tipSure = sure;

        column.update(shooter.pushing(), inv.held(), now);
        intake.update(intakeMode, inv, alliance, now);
    }

    public double aimTurn() {
        return lock.turn(aim.headingTarget, follower.getPose().getHeading(), now);
    }

    public boolean tipSureJustNow() {
        return tipSureEdge;
    }

    public double gramsToTip() {
        return Math.max(0, RobotConfig.TIP_SURE_G - gramsInUpCell);
    }

    public long now() {
        return now;
    }

    public void save() {
        PoseStore.pose = follower.getPose();
        PoseStore.alliance = alliance;
        PoseStore.redUp = hive.upSide(Alliance.RED);
        PoseStore.blueUp = hive.upSide(Alliance.BLUE);
        PoseStore.held = inv.snapshot();
        PoseStore.gramsInUpCell = gramsInUpCell;
        PoseStore.savedAtMs = Hub.nowMs();
    }

    public void restore() {
        hive.set(Alliance.RED, PoseStore.redUp);
        hive.set(Alliance.BLUE, PoseStore.blueUp);
        inv.preload(PoseStore.held);
        gramsInUpCell = PoseStore.gramsInUpCell;
    }

    public void report(Telemetry t) {
        Pose p = follower.getPose();
        t.addData("loop", "%.1f ms (worst %.0f)   battery %.2f V", hub.loopMs(), hub.worstLoopMs(), hub.volts());
        t.addData("pose", "x %.1f  y %.1f  h %.1f deg", Field.x(p), Field.y(p), Math.toDegrees(p.getHeading()));
        t.addData("held", "%d/%d  %s", inv.total(), RobotConfig.MAX_HELD, inv.describe());
        t.addData("hive", "%s up-CELL on the %s side  (%s, %d tags)", alliance,
                hive.upSide(alliance) > 0 ? "far" : "audience",
                hive.connected() ? (hive.seesTags(now) ? "seen" : "not seen") : "no limelight", hive.tagsSeen());
        t.addData("cell", "%.0f g banked, %.0f g to a sure TIP", gramsInUpCell, gramsToTip());
        t.addData("shot", "range %.1f in  bearing %.0f deg  pitch %.1f  %.2f m/s  %s",
                aim.rangeIn, aim.bearingDeg, aim.pitchDeg, aim.speedMps, aim.shootable ? "in window" : "OUT OF WINDOW");
        t.addData("aim", "heading error %.1f deg  %s", aim.headingErrorDeg, aim.aimed() ? "AIMED" : "");
        t.addData("flywheel", "%.0f / %.0f rpm  %s  power %.2f", shooter.flywheel.wheelRpm(), shooter.flywheel.targetRpm(),
                shooter.flywheel.ready() ? "READY" : "", shooter.flywheel.power());
        t.addData("trim", "speed x%.2f  pitch %+.1f deg", shooter.speedTrim, shooter.pitchTrimDeg);
        t.addData("intake", "front %s  back %s  jams %d/%d", intake.front.state(), intake.back.state(), intake.front.jams(), intake.back.jams());
        t.addData("column", "%s%s  jams %d  dry fires %d", column.staged() ? "ball staged" : "nothing staged",
                column.stuck() ? "  STUCK" : "", column.jams(), shooter.dryFires());
    }
}
