package org.firstinspires.ftc.teamcode.pedroPathing.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class CachedMotor {
    public final DcMotorEx motor;
    private double lastPower = Double.NaN;
    private double amps;
    private long ampsAtMs;

    public CachedMotor(HardwareMap hw, String name, boolean reversed, boolean brake) {
        motor = hw.get(DcMotorEx.class, name);
        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double p) {
        p = Math.max(-1, Math.min(1, p));
        boolean changed = Double.isNaN(lastPower) || Math.abs(p - lastPower) > 0.005 || (p == 0 && lastPower != 0);
        if (changed) {
            motor.setPower(p);
            lastPower = p;
        }
    }

    public double power() {
        return Double.isNaN(lastPower) ? 0 : lastPower;
    }

    public double velocity() {
        return motor.getVelocity();
    }

    public double amps(long nowMs, long everyMs) {
        if (nowMs - ampsAtMs >= everyMs) {
            amps = motor.getCurrent(CurrentUnit.AMPS);
            ampsAtMs = nowMs;
        }
        return amps;
    }
}
