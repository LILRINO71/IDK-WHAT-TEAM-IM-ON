package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.robot.Ball;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.Column;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.Flywheel;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.Hub;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.ShotTable;

@TeleOp(name = "Shooter Tuner", group = "BIOBUZZ")
public class ShooterTuner extends OpMode {
    private Hub hub;
    private Shooter shooter;
    private Column column;
    private boolean tableMode = true;
    private double manualRpm = 1100, manualPitch = 70;
    private double rangeIn = 40, bearingDeg = 0;
    private boolean nectar;
    private long shotAtMs = -1;
    private double lastRecoveryMs, lastDipPct;
    private int shots;
    private final boolean[] last = new boolean[16];

    @Override
    public void init() {
        hub = new Hub(hardwareMap);
        shooter = new Shooter(hardwareMap);
        try {
            column = new Column(hardwareMap);
        } catch (Exception e) {
            column = null;
        }
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void loop() {
        hub.update();
        long now = Hub.nowMs();
        if (column != null) column.read();

        if (edge(0, gamepad1.y)) tableMode = !tableMode;
        if (edge(1, gamepad1.x)) nectar = !nectar;
        if (tableMode) {
            if (edge(2, gamepad1.dpad_up)) rangeIn += 2.5;
            if (edge(3, gamepad1.dpad_down)) rangeIn -= 2.5;
            if (edge(4, gamepad1.dpad_right)) bearingDeg = Math.min(75, bearingDeg + 10);
            if (edge(5, gamepad1.dpad_left)) bearingDeg = Math.max(0, bearingDeg - 10);
        } else {
            if (edge(2, gamepad1.dpad_up)) manualRpm += 25;
            if (edge(3, gamepad1.dpad_down)) manualRpm -= 25;
            if (edge(4, gamepad1.dpad_right)) manualPitch += 1;
            if (edge(5, gamepad1.dpad_left)) manualPitch -= 1;
        }
        if (edge(6, gamepad1.right_bumper)) RobotConfig.EXIT_EFFICIENCY += 0.005;
        if (edge(7, gamepad1.left_bumper)) RobotConfig.EXIT_EFFICIENCY -= 0.005;
        boolean spin = !gamepad1.b;

        Ball ball = nectar ? Ball.RED_NECTAR : Ball.POLLEN;
        double pitch = tableMode ? ShotTable.pitchDeg(rangeIn, bearingDeg, ball) : manualPitch;
        double speed = tableMode ? ShotTable.speedMps(rangeIn, bearingDeg, ball) : 0;
        if (!spin) shooter.stop();
        else if (tableMode) shooter.aimAt(pitch, speed, now);
        else {
            shooter.setPitch(pitch, now);
            shooter.flywheel.setWheelRpm(manualRpm);
        }

        boolean staged = column == null || column.staged();
        shooter.update(gamepad1.a, true, staged, hub.volts(), now);
        if (column != null) column.update(shooter.pushing(), gamepad1.right_trigger > 0.2 ? 1 : 0, now);
        if (shooter.firedThisLoop()) {
            shots++;
            shotAtMs = now;
            lastDipPct = shooter.flywheel.lastDip() * 100;
        }
        if (shotAtMs > 0 && shooter.flywheel.ready()) {
            lastRecoveryMs = now - shotAtMs;
            shotAtMs = -1;
        }

        telemetry.addData("mode", tableMode ? "TABLE (y for manual)" : "MANUAL (y for table)");
        if (tableMode) telemetry.addData("table", "range %.1f in (dpad up/down)  bearing %.0f deg (dpad l/r)  %s (x)",
                rangeIn, bearingDeg, nectar ? "NECTAR" : "POLLEN");
        else telemetry.addData("manual", "wheel %.0f rpm (dpad up/down)  pitch %.0f deg (dpad l/r)", manualRpm, manualPitch);
        telemetry.addData("shot", "pitch %.1f deg  exit %.2f m/s -> wheel %.0f rpm", pitch, speed, Flywheel.wheelRpmFor(speed));
        telemetry.addData("flywheel", "%.0f / %.0f rpm  power %.2f  %s", shooter.flywheel.wheelRpm(), shooter.flywheel.targetRpm(),
                shooter.flywheel.power(), shooter.flywheel.ready() ? "READY" : "");
        telemetry.addData("exit efficiency", "%.3f  (bumpers; copy into RobotConfig when shots land)", RobotConfig.EXIT_EFFICIENCY);
        telemetry.addData("last shot", "#%d  dip %.1f %%  back to ready in %.0f ms", shots, lastDipPct, lastRecoveryMs);
        telemetry.addData("column", column == null ? "not configured" : (column.staged() ? "ball staged" : "empty (rt runs the column)"));
        telemetry.addData("battery", "%.2f V   loop %.1f ms", hub.volts(), hub.loopMs());
        telemetry.addLine("a fires one ball, hold b to stop the wheel");
        telemetry.update();
    }

    private boolean edge(int i, boolean now) {
        boolean e = now && !last[i];
        last[i] = now;
        return e;
    }
}
