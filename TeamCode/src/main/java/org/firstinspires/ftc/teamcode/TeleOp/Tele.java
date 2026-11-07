package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "tele", group = "Sister team")
public class Tele extends LinearOpMode {

    // shooter speed in motor rpm, change it with gamepad2 dpad left/right and copy the number here
    public static double SHOOTER_RPM = 4500;
    public static double RPM_STEP = 100;
    // 28 for a bare goBILDA or REV motor
    public static double TICKS_PER_REV = 28;
    // copy these from the Shooter PID Tuner
    public static double SHOOTER_P = 10;
    public static double SHOOTER_I = 0;
    public static double SHOOTER_F = 12;
    // shooter counts as ready when it is this close to the target
    public static double READY_BAND = 0.03;

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");
        DcMotor intake = hardwareMap.dcMotor.get("Intake");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        Servo gate = hardwareMap.get(Servo.class, "gate");
        Servo adj = hardwareMap.get(Servo.class, "adj");

        // both right side wheels have to be reversed, only fr was before
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor m : new DcMotor[] {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, 0, SHOOTER_F);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
        imu.resetYaw();

        double rpm = SHOOTER_RPM;
        boolean lastLeft = false, lastRight = false;

        telemetry.addLine("ready");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.y) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // intake always on, hold A to reverse, hold LT to stop
            if (gamepad2.a) {
                intake.setPower(-1);
            } else if (gamepad2.left_trigger_pressed) {
                intake.setPower(0);
            } else {
                intake.setPower(1);
            }

            if (gamepad2.dpad_right && !lastRight) rpm += RPM_STEP;
            if (gamepad2.dpad_left && !lastLeft) rpm = Math.max(0, rpm - RPM_STEP);
            lastLeft = gamepad2.dpad_left;
            lastRight = gamepad2.dpad_right;

            // hold RT to spin the shooter, the encoder keeps it at the same rpm every shot
            boolean spin = gamepad2.right_trigger_pressed;
            shooter.setVelocity(spin ? rpm * TICKS_PER_REV / 60 : 0);
            double shooterRpm = shooter.getVelocity() * 60 / TICKS_PER_REV;
            boolean ready = spin && Math.abs(shooterRpm - rpm) < rpm * READY_BAND;
            if (spin && !ready) gamepad2.rumble(50);

            if (gamepad1.a) {
                gate.setPosition(.6);
            } else {
                gate.setPosition(.75);
            }

            if (gamepad2.dpad_up) {
                adj.setPosition(.8);
            }
            if (gamepad2.dpad_down) {
                adj.setPosition(.5);
            }

            telemetry.addData("shooter", "%s  %.0f / %.0f rpm", ready ? "READY" : (spin ? "spinning up" : "off"), shooterRpm, rpm);
            telemetry.addData("put in the code", "SHOOTER_RPM = %.0f", rpm);
            telemetry.addData("gate", gate.getPosition());
            telemetry.addData("adj", adj.getPosition());
            telemetry.addData("heading", "%.1f", Math.toDegrees(botHeading));
            telemetry.update();
        }
    }
}
