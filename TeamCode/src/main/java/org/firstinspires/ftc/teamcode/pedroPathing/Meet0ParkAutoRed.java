package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Meet0 Auto RED - Leave + Park", group = "1 Meet 0", preselectTeleOp = "Meet0 #1 MAIN TeleOp ★")
public class Meet0ParkAutoRed extends LinearOpMode {

    // tune these on the field with the dpad during INIT, then copy them in here
    public static double DELAY_SECONDS = 0;
    public static double FORWARD_SECONDS = 0.3;
    public static double STRAFE_SECONDS = 0.8;
    public static double DRIVE_POWER = 0.4;
    public static boolean STRAFE_LEFT = true;
    public static boolean HEADING_HOLD = true;
    public static double HEADING_P = 0.02;

    private static final String[] SETTINGS = {"DELAY_SECONDS", "FORWARD_SECONDS", "STRAFE_SECONDS",
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
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx m : new DcMotorEx[] {frontLeft, backLeft, frontRight, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        // which way the control hub logo and usb ports face on the robot
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
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

            telemetry.addLine(alliance() + " - Leave + Park");
            telemetry.addLine("start: back on your alliance wall, facing the field, loading zone just to the robot's "
                    + (STRAFE_LEFT ? "left" : "right") + " (not on its tape)");
            telemetry.addLine("dpad up/down pick, dpad left/right change, copy the numbers into the code");
            for (int i = 0; i < SETTINGS.length; i++) {
                telemetry.addLine((i == selected ? "> " : "   ") + SETTINGS[i] + " = " + value(i));
            }
            telemetry.addData("heading", "%.1f deg", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("battery", "%.1f V", battery.getVoltage());
            telemetry.update();
        }
        if (isStopRequested()) return;

        imu.resetYaw();
        double volts = battery.getVoltage();
        powerScale = volts > 6 ? 12.0 / volts : 1;

        hold(DELAY_SECONDS);
        drive(1, 0, FORWARD_SECONDS);
        hold(0.25);
        drive(0, STRAFE_LEFT ? -1 : 1, STRAFE_SECONDS);

        telemetry.addLine("parked");
        telemetry.update();
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
            case 0: DELAY_SECONDS = step(DELAY_SECONDS, 0.5, dir, 0, 25); break;
            case 1: FORWARD_SECONDS = step(FORWARD_SECONDS, 0.05, dir, 0, 3); break;
            case 2: STRAFE_SECONDS = step(STRAFE_SECONDS, 0.05, dir, 0, 5); break;
            case 3: DRIVE_POWER = step(DRIVE_POWER, 0.05, dir, 0.1, 1); break;
            case 4: STRAFE_LEFT = !STRAFE_LEFT; break;
            default: HEADING_HOLD = !HEADING_HOLD; break;
        }
    }

    private String value(int i) {
        switch (i) {
            case 0: return String.format("%.1f", DELAY_SECONDS);
            case 1: return String.format("%.2f", FORWARD_SECONDS);
            case 2: return String.format("%.2f", STRAFE_SECONDS);
            case 3: return String.format("%.2f", DRIVE_POWER);
            case 4: return String.valueOf(STRAFE_LEFT);
            default: return String.valueOf(HEADING_HOLD);
        }
    }

    private static double step(double v, double size, int dir, double min, double max) {
        double next = Math.round(v / size + dir) * size;
        return Math.max(min, Math.min(max, next));
    }
}
