package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Set;

@TeleOp(name = "Full Robot #3 Device Test", group = "0 Full Robot")
public class FullRobotDeviceTest extends LinearOpMode {

    // same names as the Full Robot TeleOp, the gate servo is the last row
    private static final String[] NAMES = {"FL", "BL", "FR", "BR", "intake", "intake2", "outtake1", "outtake2"};
    private static final int GATE = NAMES.length;
    private static final int ROWS = NAMES.length + 1;

    // a flip here carries over to the TeleOps until the app restarts
    private static boolean reversed(int i) {
        switch (i) {
            case 0: return Meet0MainTeleOp.FL_REVERSE;
            case 1: return Meet0MainTeleOp.BL_REVERSE;
            case 2: return Meet0MainTeleOp.FR_REVERSE;
            case 3: return Meet0MainTeleOp.BR_REVERSE;
            case 4: return FullRobotTeleOp.INTAKE_REVERSE;
            case 5: return FullRobotTeleOp.INTAKE2_REVERSE;
            case 6: return FullRobotTeleOp.OUTTAKE1_REVERSE;
            default: return FullRobotTeleOp.OUTTAKE2_REVERSE;
        }
    }

    private static void flip(int i) {
        switch (i) {
            case 0: Meet0MainTeleOp.FL_REVERSE = !Meet0MainTeleOp.FL_REVERSE; break;
            case 1: Meet0MainTeleOp.BL_REVERSE = !Meet0MainTeleOp.BL_REVERSE; break;
            case 2: Meet0MainTeleOp.FR_REVERSE = !Meet0MainTeleOp.FR_REVERSE; break;
            case 3: Meet0MainTeleOp.BR_REVERSE = !Meet0MainTeleOp.BR_REVERSE; break;
            case 4: FullRobotTeleOp.INTAKE_REVERSE = !FullRobotTeleOp.INTAKE_REVERSE; break;
            case 5: FullRobotTeleOp.INTAKE2_REVERSE = !FullRobotTeleOp.INTAKE2_REVERSE; break;
            case 6: FullRobotTeleOp.OUTTAKE1_REVERSE = !FullRobotTeleOp.OUTTAKE1_REVERSE; break;
            default: FullRobotTeleOp.OUTTAKE2_REVERSE = !FullRobotTeleOp.OUTTAKE2_REVERSE; break;
        }
    }

    @Override
    public void runOpMode() {
        DcMotorEx[] motors = new DcMotorEx[NAMES.length];
        String[] ports = new String[ROWS];
        for (int i = 0; i < NAMES.length; i++) {
            motors[i] = hardwareMap.tryGet(DcMotorEx.class, NAMES[i]);
            if (motors[i] == null) {
                ports[i] = "NOT IN CONFIG";
                continue;
            }
            motors[i].setDirection(FullRobotTeleOp.direction(reversed(i)));
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Set<String> hub = hardwareMap.getNamesOf(motors[i].getController());
            ports[i] = (hub.isEmpty() ? motors[i].getController().getConnectionInfo() : hub.iterator().next())
                    + " port " + motors[i].getPortNumber();
        }
        Servo gate = hardwareMap.tryGet(Servo.class, "gate");
        if (gate == null) {
            ports[GATE] = "NOT IN CONFIG";
        } else {
            Set<String> hub = hardwareMap.getNamesOf(gate.getController());
            ports[GATE] = (hub.isEmpty() ? gate.getController().getConnectionInfo() : hub.iterator().next())
                    + " servo port " + gate.getPortNumber();
        }

        while (!isStarted() && !isStopRequested()) {
            for (int i = 0; i < ROWS; i++) {
                telemetry.addData(i == GATE ? "gate" : NAMES[i], ports[i]);
            }
            telemetry.update();
        }

        int selected = 0;
        double power = 0.5;
        // the gate stays where it is until A or B is pressed on its row
        boolean gateMoved = false, gateAtOpen = false;
        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false, lastY = false;

        while (opModeIsActive()) {
            if (gamepad1.dpad_down && !lastDown) selected = (selected + 1) % ROWS;
            if (gamepad1.dpad_up && !lastUp) selected = (selected + ROWS - 1) % ROWS;
            int step = (gamepad1.dpad_right && !lastRight ? 1 : 0) - (gamepad1.dpad_left && !lastLeft ? 1 : 0);
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;

            if (selected == GATE) {
                if (gamepad1.a) {
                    gateAtOpen = false;
                    gateMoved = true;
                }
                if (gamepad1.b) {
                    gateAtOpen = true;
                    gateMoved = true;
                }
                if (step != 0 && gateMoved) {
                    double moved = Math.max(0, Math.min(1, Math.round((gateAtOpen ? FullRobotTeleOp.GATE_OPEN : FullRobotTeleOp.GATE_CLOSED) * 50 + step) / 50.0));
                    if (gateAtOpen) FullRobotTeleOp.GATE_OPEN = moved;
                    else FullRobotTeleOp.GATE_CLOSED = moved;
                }
                if (gate != null && gateMoved) gate.setPosition(gateAtOpen ? FullRobotTeleOp.GATE_OPEN : FullRobotTeleOp.GATE_CLOSED);
            } else {
                if (step != 0) power = Math.max(0.1, Math.min(1.0, Math.round(power * 10 + step) / 10.0));
                if (gamepad1.y && !lastY && motors[selected] != null) {
                    flip(selected);
                    motors[selected].setDirection(FullRobotTeleOp.direction(reversed(selected)));
                }
            }
            lastY = gamepad1.y;

            for (int i = 0; i < NAMES.length; i++) {
                if (motors[i] == null) continue;
                double p = 0;
                if (i == selected && gamepad1.a) p = power;
                if (i == selected && gamepad1.b) p = -power;
                if (i < 4 && selected != GATE && gamepad1.x) p = power;
                // RB should spin the robot right: left wheels forward, right wheels back
                if (i < 4 && gamepad1.right_bumper) p = i < 2 ? power : -power;
                motors[i].setPower(p);
            }

            telemetry.addLine("dpad up/down pick, dpad left/right power");
            telemetry.addLine("hold A forward, B backward, Y flip it, X all 4 wheels forward, RB turn right");
            telemetry.addLine("A should: roll a wheel forward, pull balls IN on an intake, throw balls UP and OUT on an outtake");
            telemetry.addLine("gate row: A closed, B open, dpad left/right moves the one it is at");
            telemetry.addData("put in Meet0MainTeleOp", "FL_REVERSE = %b, BL_REVERSE = %b, FR_REVERSE = %b, BR_REVERSE = %b",
                    Meet0MainTeleOp.FL_REVERSE, Meet0MainTeleOp.BL_REVERSE, Meet0MainTeleOp.FR_REVERSE, Meet0MainTeleOp.BR_REVERSE);
            telemetry.addData("put in FullRobotTeleOp", "INTAKE_REVERSE = %b, INTAKE2_REVERSE = %b, OUTTAKE1_REVERSE = %b, OUTTAKE2_REVERSE = %b",
                    FullRobotTeleOp.INTAKE_REVERSE, FullRobotTeleOp.INTAKE2_REVERSE, FullRobotTeleOp.OUTTAKE1_REVERSE, FullRobotTeleOp.OUTTAKE2_REVERSE);
            telemetry.addData("put in FullRobotTeleOp", "GATE_CLOSED = %.2f, GATE_OPEN = %.2f", FullRobotTeleOp.GATE_CLOSED, FullRobotTeleOp.GATE_OPEN);
            telemetry.addData("power", "%.1f", power);
            for (int i = 0; i < NAMES.length; i++) {
                String amps = motors[i] == null ? "-" : String.format("%.2f A", motors[i].getCurrent(CurrentUnit.AMPS));
                String dir = reversed(i) ? "REVERSE" : "FORWARD";
                telemetry.addData((i == selected ? "> " : "  ") + NAMES[i], "%s   %s   %s", ports[i], amps, dir);
            }
            telemetry.addData((selected == GATE ? "> " : "  ") + "gate", "%s   %s", ports[GATE],
                    !gateMoved ? "not moved yet" : (gateAtOpen ? "at OPEN" : "at CLOSED"));
            telemetry.update();
        }
    }
}
