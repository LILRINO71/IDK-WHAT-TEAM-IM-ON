package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Shooter PID Tuner", group = "Sister team")
public class ShooterPIDTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        // same direction as the tele
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(Tele.SHOOTER_P, Tele.SHOOTER_I, 0, Tele.SHOOTER_F);

        String[] names = {"SHOOTER_RPM", "SHOOTER_P", "SHOOTER_I", "SHOOTER_F"};
        int sel = 0;
        boolean on = false;
        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;
        double low = 99999, high = 0;

        telemetry.addLine("A on, B off, dpad up/down pick, left/right change, X clear low/high");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) on = true;
            if (gamepad1.b) on = false;
            if (gamepad1.dpad_up && !lastUp) sel = (sel + names.length - 1) % names.length;
            if (gamepad1.dpad_down && !lastDown) sel = (sel + 1) % names.length;
            int dir = 0;
            if (gamepad1.dpad_right && !lastRight) dir = 1;
            if (gamepad1.dpad_left && !lastLeft) dir = -1;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;

            if (dir != 0) {
                switch (sel) {
                    case 0: Tele.SHOOTER_RPM = Math.max(0, Tele.SHOOTER_RPM + 100 * dir); break;
                    case 1: Tele.SHOOTER_P = Math.max(0, Tele.SHOOTER_P + dir); break;
                    case 2: Tele.SHOOTER_I = Math.max(0, Math.round((Tele.SHOOTER_I + 0.1 * dir) * 10) / 10.0); break;
                    default: Tele.SHOOTER_F = Math.max(0, Tele.SHOOTER_F + 0.5 * dir); break;
                }
                shooter.setVelocityPIDFCoefficients(Tele.SHOOTER_P, Tele.SHOOTER_I, 0, Tele.SHOOTER_F);
            }

            shooter.setVelocity(on ? Tele.SHOOTER_RPM * Tele.TICKS_PER_REV / 60 : 0);
            double rpm = shooter.getVelocity() * 60 / Tele.TICKS_PER_REV;

            if (gamepad1.x || !on) {
                low = 99999;
                high = 0;
            }
            if (on) {
                low = Math.min(low, rpm);
                high = Math.max(high, rpm);
            }

            telemetry.addLine("A on, B off, dpad up/down pick, left/right change, X clear low/high");
            double[] values = {Tele.SHOOTER_RPM, Tele.SHOOTER_P, Tele.SHOOTER_I, Tele.SHOOTER_F};
            for (int i = 0; i < names.length; i++) {
                telemetry.addLine((i == sel ? "> " : "   ") + names[i] + " = " + values[i]);
            }
            telemetry.addData("rpm", "%.0f  (target %.0f)", rpm, on ? Tele.SHOOTER_RPM : 0);
            telemetry.addData("swing", on ? String.format("low %.0f  high %.0f", low, high) : "-");
            telemetry.addData("put in Tele", "SHOOTER_RPM = %.0f, P = %.1f, I = %.1f, F = %.1f",
                    Tele.SHOOTER_RPM, Tele.SHOOTER_P, Tele.SHOOTER_I, Tele.SHOOTER_F);
            telemetry.update();
        }
    }
}
