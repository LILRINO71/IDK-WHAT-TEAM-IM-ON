package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.BallSensor;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.RobotConfig;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Hardware Check", group = "BIOBUZZ")
public class HardwareCheck extends OpMode {
    private final List<String> motorNames = new ArrayList<>();
    private final List<DcMotorEx> motors = new ArrayList<>();
    private final List<String> servoNames = new ArrayList<>();
    private final List<Servo> servos = new ArrayList<>();
    private final double[] servoPos = new double[8];
    private RevColorSensorV3 frontColor, backColor;
    private DigitalChannel columnIn, columnTop;
    private final List<String> missing = new ArrayList<>();
    private int selected;
    private final boolean[] last = new boolean[8];

    @Override
    public void init() {
        for (String n : new String[]{"leftFront", "leftRear", "rightFront", "rightRear",
                RobotConfig.FRONT_INTAKE, RobotConfig.BACK_INTAKE, RobotConfig.COLUMN, RobotConfig.FLYWHEEL}) {
            DcMotorEx m = hardwareMap.tryGet(DcMotorEx.class, n);
            if (m == null) missing.add("motor " + n);
            else {
                motorNames.add(n);
                motors.add(m);
            }
        }
        for (String n : new String[]{RobotConfig.PITCH_SERVO, RobotConfig.STOPPER_SERVO,
                RobotConfig.LEFT_JAM_SERVO, RobotConfig.RIGHT_JAM_SERVO}) {
            Servo s = hardwareMap.tryGet(Servo.class, n);
            if (s == null) missing.add("servo " + n);
            else {
                servoNames.add(n);
                servos.add(s);
            }
        }
        frontColor = hardwareMap.tryGet(RevColorSensorV3.class, RobotConfig.FRONT_COLOR);
        backColor = hardwareMap.tryGet(RevColorSensorV3.class, RobotConfig.BACK_COLOR);
        columnIn = hardwareMap.tryGet(DigitalChannel.class, RobotConfig.COLUMN_IN);
        columnTop = hardwareMap.tryGet(DigitalChannel.class, RobotConfig.COLUMN_TOP);
        if (frontColor == null) missing.add("color " + RobotConfig.FRONT_COLOR);
        if (backColor == null) missing.add("color " + RobotConfig.BACK_COLOR);
        if (columnIn == null) missing.add("digital " + RobotConfig.COLUMN_IN);
        if (columnTop == null) missing.add("digital " + RobotConfig.COLUMN_TOP);
        if (hardwareMap.tryGet(GoBildaPinpointDriver.class, "pinpoint") == null) missing.add("pinpoint");
        if (hardwareMap.tryGet(Limelight3A.class, RobotConfig.LIMELIGHT) == null) missing.add("limelight");
        if (columnIn != null) columnIn.setMode(DigitalChannel.Mode.INPUT);
        if (columnTop != null) columnTop.setMode(DigitalChannel.Mode.INPUT);
        for (int i = 0; i < servoPos.length; i++) servoPos[i] = 0.5;
    }

    @Override
    public void loop() {
        int count = motors.size() + servos.size();
        if (count > 0) {
            if (edge(0, gamepad1.dpad_down)) selected = (selected + 1) % count;
            if (edge(1, gamepad1.dpad_up)) selected = (selected + count - 1) % count;
        }
        for (int i = 0; i < motors.size(); i++) {
            double p = 0;
            if (i == selected) {
                if (gamepad1.a) p = 0.3;
                if (gamepad1.b) p = -0.3;
            }
            motors.get(i).setPower(p);
        }
        int s = selected - motors.size();
        if (s >= 0 && s < servos.size()) {
            servoPos[s] = Math.max(0, Math.min(1, servoPos[s] - gamepad1.left_stick_y * 0.004));
            if (edge(2, gamepad1.x)) servoPos[s] = 0.5;
            servos.get(s).setPosition(servoPos[s]);
        }

        telemetry.addData("missing", missing.isEmpty() ? "nothing" : missing.toString());
        telemetry.addLine("dpad picks a device, a/b run a motor at +/-30%, left stick moves a servo");
        for (int i = 0; i < motors.size(); i++) {
            DcMotorEx m = motors.get(i);
            telemetry.addData((i == selected ? "> " : "  ") + motorNames.get(i), "ticks %d  vel %.0f/s  %.1f A",
                    m.getCurrentPosition(), m.getVelocity(), i == selected ? m.getCurrent(CurrentUnit.AMPS) : 0.0);
        }
        for (int i = 0; i < servos.size(); i++) {
            telemetry.addData((motors.size() + i == selected ? "> " : "  ") + servoNames.get(i), "position %.3f", servoPos[i]);
        }
        color("front color", frontColor);
        color("back color", backColor);
        if (columnIn != null) telemetry.addData("column in", columnIn.getState() == RobotConfig.BEAM_BROKEN_IS_LOW ? "clear" : "BROKEN");
        if (columnTop != null) telemetry.addData("column top", columnTop.getState() == RobotConfig.BEAM_BROKEN_IS_LOW ? "clear" : "BROKEN");
        telemetry.update();
    }

    private void color(String label, RevColorSensorV3 c) {
        if (c == null) return;
        double d = c.getDistance(DistanceUnit.MM);
        NormalizedRGBA rgba = c.getNormalizedColors();
        double[] hs = BallSensor.hueSat(rgba.red, rgba.green, rgba.blue);
        telemetry.addData(label, "%.0f mm  hue %.0f  sat %.2f  -> %s", d, hs[0], hs[1],
                d <= RobotConfig.BALL_PRESENT_MM ? BallSensor.classify(hs[0], hs[1]) : "no ball");
    }

    private boolean edge(int i, boolean now) {
        boolean e = now && !last[i];
        last[i] = now;
        return e;
    }
}
