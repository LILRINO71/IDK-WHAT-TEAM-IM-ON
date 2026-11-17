package org.firstinspires.ftc.teamcode.pedroPathing.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class JammingWheels {
    private final Servo left, right;
    private boolean down;

    public JammingWheels(HardwareMap hw) {
        left = hw.get(Servo.class, RobotConfig.LEFT_JAM_SERVO);
        right = hw.get(Servo.class, RobotConfig.RIGHT_JAM_SERVO);
        right.setDirection(Servo.Direction.REVERSE);
        set(false);
    }

    public void set(boolean wantDown) {
        down = wantDown;
        double p = wantDown ? RobotConfig.JAM_DOWN : RobotConfig.JAM_UP;
        left.setPosition(p);
        right.setPosition(p);
    }

    public void toggle() {
        set(!down);
    }

    public boolean down() {
        return down;
    }
}
