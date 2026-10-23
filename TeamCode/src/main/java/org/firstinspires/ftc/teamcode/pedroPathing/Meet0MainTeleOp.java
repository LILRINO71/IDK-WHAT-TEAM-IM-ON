package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Meet0 #1 MAIN TeleOp ★", group = "1 Meet 0")
public class Meet0MainTeleOp extends LinearOpMode {

    public static double STICK_DEADBAND = 0.05;

    // shooter speed without encoders, as power on a 12 V battery (0.55 is about 3300 rpm)
    public static double SHOOTER_POWER = 0.55;
    public static double POWER_STEP = 0.01;
    // shooter speed with encoders, in motor rpm
    public static double SHOOTER_RPM = 3300;
    public static double RPM_STEP = 50;
    // extra shooter speed while LB is held for a NECTAR, copy the number the driver hub shows
    public static double NECTAR_BOOST = 0.04;
    public static double BOOST_STEP = 0.01;

    // false = RB only spins the shooter and you feed with RT yourself
    public static boolean AUTO_FEED = true;
    public static double SPIN_UP_SECONDS = 1.0;
    public static double SETTLE_SECONDS = 0.3;
    public static double READY_BAND = 0.03;
    public static double TICKS_PER_REV = 28;
    public static double SHOOTER_P = 12;
    public static double SHOOTER_I = 0.5;
    public static double SHOOTER_D = 0;
    public static double SHOOTER_F = 13;

    // 435 rpm transfer motor (config name "transfer"), runs while feeding and backwards on eject
    public static double TRANSFER_POWER = 1.0;
    public static boolean TRANSFER_REVERSED = false;

    protected boolean useEncoders() {
        return false;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "BL");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "BR");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        DcMotorEx outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        DcMotorEx transfer = hardwareMap.tryGet(DcMotorEx.class, "transfer");
        if (transfer != null) {
            transfer.setDirection(TRANSFER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        // flip one of these if the two outtake motors fight each other
        outtake1.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[] {frontLeft, backLeft, frontRight, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        for (DcMotorEx m : new DcMotorEx[] {outtake1, outtake2}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            if (useEncoders()) {
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                m.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
            } else {
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // which way the control hub logo and usb ports face on the robot
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        double rpm = SHOOTER_RPM, power = SHOOTER_POWER, boost = NECTAR_BOOST;
        boolean shooterOn = false;
        boolean lastA = false, lastUp = false, lastDown = false, lastLeft = false, lastRight = false, lastBack = false;
        double lastPower = 0, waitFor = SPIN_UP_SECONDS;
        ElapsedTime sinceChange = new ElapsedTime();
        String mode = useEncoders() ? "with encoders" : "no encoders";

        telemetry.addData("MAIN TeleOp", mode);
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = Math.pow(applyDeadband(-gamepad1.left_stick_y), 3);
            double x = Math.pow(applyDeadband(gamepad1.left_stick_x), 3);
            double rx = Math.pow(applyDeadband(gamepad1.right_stick_x), 3);
            if (gamepad1.y) imu.resetYaw();
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) * 1.1;
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            if (gamepad1.a && !lastA) shooterOn = !shooterOn;
            if (gamepad1.dpad_up && !lastUp) {
                if (useEncoders()) rpm += RPM_STEP;
                else power = Math.min(1.0, power + POWER_STEP);
            }
            if (gamepad1.dpad_down && !lastDown) {
                if (useEncoders()) rpm = Math.max(0, rpm - RPM_STEP);
                else power = Math.max(0, power - POWER_STEP);
            }
            if (gamepad1.dpad_right && !lastRight) boost += BOOST_STEP;
            if (gamepad1.dpad_left && !lastLeft) boost = Math.max(0, boost - BOOST_STEP);
            if (gamepad1.back && !lastBack) {
                rpm = SHOOTER_RPM;
                power = SHOOTER_POWER;
                boost = NECTAR_BOOST;
            }
            lastA = gamepad1.a;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;
            lastBack = gamepad1.back;

            boolean shoot = gamepad1.right_bumper || gamepad2.right_bumper;
            boolean nectar = gamepad1.left_bumper || gamepad2.left_bumper;
            boolean spin = shooterOn || shoot;
            double scale = nectar ? 1 + boost : 1;
            double volts = battery.getVoltage();
            boolean ready;
            String shooter;
            if (useEncoders()) {
                double target = rpm * scale;
                double tps = spin ? target * TICKS_PER_REV / 60 : 0;
                outtake1.setVelocity(tps);
                outtake2.setVelocity(tps);
                double rpm1 = outtake1.getVelocity() * 60 / TICKS_PER_REV;
                double rpm2 = outtake2.getVelocity() * 60 / TICKS_PER_REV;
                ready = spin && Math.abs(rpm1 - target) < target * READY_BAND && Math.abs(rpm2 - target) < target * READY_BAND;
                shooter = String.format("target %.0f rpm%s  1: %.0f  2: %.0f", spin ? target : 0, nectar ? " (NECTAR)" : "", rpm1, rpm2);
            } else {
                double target = spin ? power * scale : 0;
                if (Math.abs(target - lastPower) > 1e-6) {
                    waitFor = lastPower == 0 ? SPIN_UP_SECONDS : SETTLE_SECONDS;
                    sinceChange.reset();
                    lastPower = target;
                }
                double out = volts > 6 ? Math.min(1.0, target * 12.0 / volts) : target;
                outtake1.setPower(out);
                outtake2.setPower(out);
                ready = spin && sinceChange.seconds() >= waitFor;
                shooter = String.format("power %.2f%s  battery %.1f V", target, nectar ? " (NECTAR)" : "", volts);
            }

            boolean feed = AUTO_FEED && shoot && ready;
            boolean eject = gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1;
            if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1 || feed) intake.setPower(1.0);
            else if (eject) intake.setPower(-1.0);
            else intake.setPower(0.0);
            if (transfer != null) transfer.setPower(feed ? TRANSFER_POWER : (eject ? -TRANSFER_POWER : 0.0));
            if (shoot && !ready) gamepad1.rumble(50);

            telemetry.addData("MAIN TeleOp", mode);
            telemetry.addData("shooter", "%s  %s", ready ? "READY" : (spin ? "spinning up" : "off"), shooter);
            telemetry.addData("transfer", transfer == null ? "not in the config" : (feed ? "feeding" : (eject ? "reverse" : "stopped")));
            if (useEncoders()) {
                telemetry.addData("speeds", "POLLEN %.0f rpm, NECTAR +%.0f%% = %.0f rpm", rpm, boost * 100, rpm * (1 + boost));
                telemetry.addData("put in the code", "SHOOTER_RPM = %.0f, NECTAR_BOOST = %.2f", rpm, boost);
            } else {
                telemetry.addData("speeds", "POLLEN power %.2f, NECTAR +%.0f%% = %.2f", power, boost * 100, power * (1 + boost));
                telemetry.addData("put in the code", "SHOOTER_POWER = %.2f, NECTAR_BOOST = %.2f", power, boost);
            }
            telemetry.addData("Heading (Deg)", "%.1f", Math.toDegrees(botHeading));
            telemetry.addData("FL / FR", "%.2f / %.2f", frontLeftPower, frontRightPower);
            telemetry.addData("BL / BR", "%.2f / %.2f", backLeftPower, backRightPower);
            telemetry.addData("controls", "RB shoot, LB NECTAR, A shooter on/off, RT intake, LT eject, Y reset heading, "
                    + "dpad up/down POLLEN speed, dpad left/right NECTAR boost, back reset");
            telemetry.update();
        }
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < STICK_DEADBAND ? 0.0 : value;
    }
}
