package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Full Robot Auto RED - Shoot + Park", group = "0 Full Robot", preselectTeleOp = "Full Robot #1 TeleOp ★")
public class FullRobotAutoRed extends LinearOpMode {

    // tune these on the field with the dpad during INIT, then copy them in here
    public static double DELAY_SECONDS = 0;
    // 0 skips the shot and only parks
    public static double SHOOT_SECONDS = 3.0;
    public static double FORWARD_SECONDS = 0.3;
    public static double STRAFE_SECONDS = 0.8;
    public static double DRIVE_POWER = 0.4;
    public static boolean STRAFE_LEFT = true;
    public static boolean HEADING_HOLD = true;
    public static double HEADING_P = 0.02;

    private static final String[] SETTINGS = {"DELAY_SECONDS", "SHOOT_SECONDS", "FORWARD_SECONDS", "STRAFE_SECONDS",
            "DRIVE_POWER", "STRAFE_LEFT", "HEADING_HOLD"};

    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private IMU imu;
    private double powerScale = 1;

    protected String alliance() {
        return "RED";
    }

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        backLeft = hardwareMap.get(DcMotorEx.class, "BL");
        frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        backRight = hardwareMap.get(DcMotorEx.class, "BR");
        Meet0MainTeleOp.setDriveDirections(frontLeft, backLeft, frontRight, backRight);
        for (DcMotorEx m : new DcMotorEx[] {frontLeft, backLeft, frontRight, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx intake2 = hardwareMap.tryGet(DcMotorEx.class, "intake2");
        DcMotorEx outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        DcMotorEx outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        Servo gate = hardwareMap.tryGet(Servo.class, "gate");
        // same as the Full Robot TeleOp
        FullRobotTeleOp.setMechanismDirections(intake, intake2, outtake1, outtake2);
        for (DcMotorEx m : new DcMotorEx[] {intake, intake2, outtake1, outtake2}) {
            if (m == null) continue;
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        // which way the control hub logo and usb ports face on the robot
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN)));
        imu.resetYaw();
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        int selected = 0;
        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;
        while (opModeInInit()) {
            if (gamepad1.dpad_up && !lastUp) selected = (selected + SETTINGS.length - 1) % SETTINGS.length;
            if (gamepad1.dpad_down && !lastDown) selected = (selected + 1) % SETTINGS.length;
            if (gamepad1.dpad_right && !lastRight) change(selected, 1);
            if (gamepad1.dpad_left && !lastLeft) change(selected, -1);
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;

            telemetry.addLine(alliance() + " - Shoot + Park");
            telemetry.addLine("start: balls loaded, the shooter side (robot's left) aimed at the HIVE, loading zone to the robot's "
                    + (STRAFE_LEFT ? "left" : "right"));
            telemetry.addLine("dpad up/down pick, dpad left/right change, copy the numbers into the code");
            for (int i = 0; i < SETTINGS.length; i++) {
                telemetry.addLine((i == selected ? "> " : "   ") + SETTINGS[i] + " = " + value(i));
            }
            if (intake2 == null) telemetry.addLine("intake2 is NOT IN CONFIG: nothing lifts the balls to the shooter");
            if (gate == null) telemetry.addLine("gate is NOT IN CONFIG");
            telemetry.addData("heading", "%.1f deg", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("battery", "%.1f V", battery.getVoltage());
            telemetry.update();
        }
        if (isStopRequested()) return;

        imu.resetYaw();
        double volts = battery.getVoltage();
        powerScale = volts > 6 ? 12.0 / volts : 1;
        double shooter = FullRobotTeleOp.SHOOTER_POWER >= 1.0 ? 1.0 : Math.min(1.0, FullRobotTeleOp.SHOOTER_POWER * powerScale);
        if (gate != null) gate.setPosition(FullRobotTeleOp.GATE_CLOSED);
        outtake1.setPower(shooter);
        outtake2.setPower(shooter);

        hold(DELAY_SECONDS);
        if (SHOOT_SECONDS > 0) {
            hold(FullRobotTeleOp.SPIN_UP_SECONDS);
            if (gate != null) gate.setPosition(FullRobotTeleOp.GATE_OPEN);
            hold(FullRobotTeleOp.GATE_SECONDS);
            intake.setPower(FullRobotTeleOp.FEED_POWER);
            if (intake2 != null) intake2.setPower(FullRobotTeleOp.FEED_POWER);
            telemetry.addLine("shooting");
            telemetry.update();
            hold(SHOOT_SECONDS);
            intake.setPower(0);
            if (intake2 != null) intake2.setPower(0);
            if (gate != null) gate.setPosition(FullRobotTeleOp.GATE_CLOSED);
        }

        drive(1, 0, FORWARD_SECONDS);
        hold(0.25);
        drive(0, STRAFE_LEFT ? -1 : 1, STRAFE_SECONDS);

        telemetry.addLine("parked, outtake on");
        telemetry.update();
        while (opModeIsActive()) {
            idle();
        }
    }

    private void drive(double y, double x, double seconds) {
        ElapsedTime timer = new ElapsedTime();
        double p = Math.min(1.0, DRIVE_POWER * powerScale);
        while (opModeIsActive() && timer.seconds() < seconds) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double rx = 0;
            if (HEADING_HOLD && Math.abs(heading) < 30) rx = Math.max(-0.2, Math.min(0.2, heading * HEADING_P));
            double fl = (y + x) * p + rx;
            double bl = (y - x) * p + rx;
            double fr = (y - x) * p - rx;
            double br = (y + x) * p - rx;
            double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br))));
            frontLeft.setPower(fl / max);
            backLeft.setPower(bl / max);
            frontRight.setPower(fr / max);
            backRight.setPower(br / max);
            telemetry.addData("heading", "%.1f deg", heading);
            telemetry.update();
        }
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    private void hold(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            idle();
        }
    }

    private void change(int i, int dir) {
        switch (i) {
            case 0: DELAY_SECONDS = step(DELAY_SECONDS, 0.5, dir, 0, 20); break;
            case 1: SHOOT_SECONDS = step(SHOOT_SECONDS, 0.5, dir, 0, 15); break;
            case 2: FORWARD_SECONDS = step(FORWARD_SECONDS, 0.05, dir, 0, 3); break;
            case 3: STRAFE_SECONDS = step(STRAFE_SECONDS, 0.05, dir, 0, 5); break;
            case 4: DRIVE_POWER = step(DRIVE_POWER, 0.05, dir, 0.1, 1); break;
            case 5: STRAFE_LEFT = !STRAFE_LEFT; break;
            default: HEADING_HOLD = !HEADING_HOLD; break;
        }
    }

    private String value(int i) {
        switch (i) {
            case 0: return String.format("%.1f", DELAY_SECONDS);
            case 1: return String.format("%.1f", SHOOT_SECONDS);
            case 2: return String.format("%.2f", FORWARD_SECONDS);
            case 3: return String.format("%.2f", STRAFE_SECONDS);
            case 4: return String.format("%.2f", DRIVE_POWER);
            case 5: return String.valueOf(STRAFE_LEFT);
            default: return String.valueOf(HEADING_HOLD);
        }
    }

    private static double step(double v, double size, int dir, double min, double max) {
        double next = Math.round(v / size + dir) * size;
        return Math.max(min, Math.min(max, next));
    }
}
