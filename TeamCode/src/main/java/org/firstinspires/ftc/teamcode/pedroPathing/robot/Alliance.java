package org.firstinspires.ftc.teamcode.pedroPathing.robot;

public enum Alliance {
    RED(-1), BLUE(1);

    public final int sign;

    Alliance(int sign) {
        this.sign = sign;
    }

    public double hiveX() {
        return sign * Field.HIVE_X;
    }

    public int startUpSide() {
        return sign;
    }

    public double driverHeading() {
        return this == RED ? 0 : Math.PI;
    }
}
