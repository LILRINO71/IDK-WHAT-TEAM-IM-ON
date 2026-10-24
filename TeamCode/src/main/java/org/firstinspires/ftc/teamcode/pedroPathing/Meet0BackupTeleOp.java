package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Meet0 #3 Backup - old team TeleOp", group = "1 Meet 0")
public class Meet0BackupTeleOp extends LinearOpMode {

    private static final double STICK_DEADBAND = 0.05;

    // ===== OUTTAKE SPEED =====
    // CHANGE SPEED HERE: 1.0 = full power, 0.5 = half power, etc. (range 0.0 to 1.0)
    private static final double OUTTAKE_POWER = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Initialize Drivetrain Motors
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "BL");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "BR");

        // Initialize 435 RPM Intake Motor
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Initialize Outtake Motors (names must match the Driver Hub config exactly)
        DcMotorEx outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        DcMotorEx outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        // Set Motor Directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        // if balls come out backwards, swap which of these two is REVERSE
        outtake1.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set Zero Power Behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Strictly run without encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 2. Initialize IMU (Strictly for Field-Centric translation, NO heading lock)
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // which way the control hub logo and usb ports face on the robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        imu.resetYaw();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // 3. Read Gamepad inputs with Deadband
            double rawY = applyDeadband(-gamepad1.left_stick_y); // Forward is positive
            double rawX = applyDeadband(gamepad1.left_stick_x);  // Right is positive
            double rawRx = applyDeadband(gamepad1.right_stick_x); // Manual rotation only (No PID lock)

            // Cubic scaling for fine driver control
            double y = Math.pow(rawY, 3);
            double x = Math.pow(rawX, 3);
            double rx = Math.pow(rawRx, 3);

            // 4. Intake Control (Right Trigger = Intake In, Left Trigger = Outtake/Unjam)
            if (gamepad1.right_trigger > 0.1) {
                intake.setPower(1.0);
            } else if (gamepad1.left_trigger > 0.1) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }

            // Outtake Control (Hold Right Bumper = run both outtake motors)
            // Speed is set by OUTTAKE_POWER at the top of the file.
            if (gamepad1.right_bumper) {
                outtake1.setPower(OUTTAKE_POWER);
                outtake2.setPower(OUTTAKE_POWER);
            } else {
                outtake1.setPower(0.0);
                outtake2.setPower(0.0);
            }

            // Reset Field-Centric "Forward" with Y Button
            if (gamepad1.y) {
                imu.resetYaw();
            }

            // 5. Field-Centric Translation Math
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Counteract imperfect strafing on Mecanum wheels
            rotX = rotX * 1.1;

            // 6. Calculate Drivetrain Motor Powers
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // 7. Send Power to Drivetrain Motors
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Telemetry
            telemetry.addData("Heading (Deg)", "%.1f", Math.toDegrees(botHeading));
            telemetry.addData("FL / FR", "%.2f / %.2f", frontLeftPower, frontRightPower);
            telemetry.addData("BL / BR", "%.2f / %.2f", backLeftPower, backRightPower);
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.addData("Outtake Power", outtake1.getPower());
            telemetry.update();
        }
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < STICK_DEADBAND ? 0.0 : value;
    }
}
