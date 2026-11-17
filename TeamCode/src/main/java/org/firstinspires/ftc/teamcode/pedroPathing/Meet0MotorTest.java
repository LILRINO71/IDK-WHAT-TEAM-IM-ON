package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Set;

@TeleOp(name = "Meet0 #4 Motor Test", group = "1 Meet 0")
public class Meet0MotorTest extends LinearOpMode {

    // same names and directions as the MAIN TeleOp
    private static final String[] NAMES = {"FL", "BL", "FR", "BR", "intake", "outtake1", "outtake2"};
    private static final boolean[] REVERSED = {true, true, false, false, true, false, false};

    // wheels use the MAIN TeleOp's directions so a flip here carries over until the app restarts
    private static boolean reversed(int i) {
        switch (i) {
            case 0: return Meet0MainTeleOp.FL_REVERSE;
            case 1: return Meet0MainTeleOp.BL_REVERSE;
            case 2: return Meet0MainTeleOp.FR_REVERSE;
            case 3: return Meet0MainTeleOp.BR_REVERSE;
            default: return REVERSED[i];
        }
    }

    private static void flip(int i) {
        switch (i) {
            case 0: Meet0MainTeleOp.FL_REVERSE = !Meet0MainTeleOp.FL_REVERSE; break;
            case 1: Meet0MainTeleOp.BL_REVERSE = !Meet0MainTeleOp.BL_REVERSE; break;
            case 2: Meet0MainTeleOp.FR_REVERSE = !Meet0MainTeleOp.FR_REVERSE; break;
            default: Meet0MainTeleOp.BR_REVERSE = !Meet0MainTeleOp.BR_REVERSE; break;
        }
    }

    @Override
    public void runOpMode() {
        DcMotorEx[] motors = new DcMotorEx[NAMES.length];
        String[] ports = new String[NAMES.length];
        for (int i = 0; i < NAMES.length; i++) {
            motors[i] = hardwareMap.tryGet(DcMotorEx.class, NAMES[i]);
            if (motors[i] == null) {
                ports[i] = "NOT IN CONFIG";
                continue;
            }
            motors[i].setDirection(reversed(i) ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Set<String> hub = hardwareMap.getNamesOf(motors[i].getController());
            ports[i] = (hub.isEmpty() ? motors[i].getController().getConnectionInfo() : hub.iterator().next())
                    + " port " + motors[i].getPortNumber();
        }

        while (!isStarted() && !isStopRequested()) {
            for (int i = 0; i < NAMES.length; i++) {
                telemetry.addData(NAMES[i], ports[i]);
            }
            telemetry.update();
        }

        int selected = 0;
        double power = 0.5;
        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false, lastY = false;

        while (opModeIsActive()) {
            if (gamepad1.dpad_down && !lastDown) selected = (selected + 1) % NAMES.length;
            if (gamepad1.dpad_up && !lastUp) selected = (selected + NAMES.length - 1) % NAMES.length;
            if (gamepad1.dpad_right && !lastRight) power = Math.min(1.0, Math.round(power * 10 + 1) / 10.0);
            if (gamepad1.dpad_left && !lastLeft) power = Math.max(0.1, Math.round(power * 10 - 1) / 10.0);
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;
            if (gamepad1.y && !lastY && selected < 4 && motors[selected] != null) {
                flip(selected);
                motors[selected].setDirection(reversed(selected) ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            }
            lastY = gamepad1.y;

            for (int i = 0; i < NAMES.length; i++) {
                if (motors[i] == null) continue;
                double p = 0;
                if (i == selected && gamepad1.a) p = power;
                if (i == selected && gamepad1.b) p = -power;
                if (i < 4 && gamepad1.x) p = power;
                // RB should spin the robot right: left wheels forward, right wheels back
                if (i < 4 && gamepad1.right_bumper) p = i < 2 ? power : -power;
                motors[i].setPower(p);
            }

            telemetry.addLine("dpad up/down pick, dpad left/right power");
            telemetry.addLine("hold A forward, B backward, X all 4 wheels forward, RB turn right");
            telemetry.addLine("X should roll the robot straight forward. a wheel going backward? pick it and press Y");
            telemetry.addData("put in Meet0MainTeleOp", "FL_REVERSE = %b, BL_REVERSE = %b, FR_REVERSE = %b, BR_REVERSE = %b",
                    Meet0MainTeleOp.FL_REVERSE, Meet0MainTeleOp.BL_REVERSE, Meet0MainTeleOp.FR_REVERSE, Meet0MainTeleOp.BR_REVERSE);
            telemetry.addData("power", "%.1f", power);
            for (int i = 0; i < NAMES.length; i++) {
                String amps = motors[i] == null ? "-" : String.format("%.2f A", motors[i].getCurrent(CurrentUnit.AMPS));
                String dir = reversed(i) ? "REVERSE" : "FORWARD";
                telemetry.addData((i == selected ? "> " : "  ") + NAMES[i], "%s   %s   %s", ports[i], amps, dir);
            }
            telemetry.update();
        }
    }
}
