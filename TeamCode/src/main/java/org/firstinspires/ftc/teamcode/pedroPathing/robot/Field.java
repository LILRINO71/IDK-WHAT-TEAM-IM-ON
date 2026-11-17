package org.firstinspires.ftc.teamcode.pedroPathing.robot;

import com.pedropathing.geometry.Pose;

public final class Field {
    public static final double HIVE_X = 12.75;
    public static final double MOUTH_OUT = 16.49;
    public static final double MOUTH_Z = 58.32;
    public static final double HALF = 70.5;
    public static final double PEDRO_OFFSET = 72;

    private Field() {
    }

    public static Pose pedro(double x, double y, double heading) {
        return new Pose(x + PEDRO_OFFSET, y + PEDRO_OFFSET, heading);
    }

    public static double x(Pose p) {
        return p.getX() - PEDRO_OFFSET;
    }

    public static double y(Pose p) {
        return p.getY() - PEDRO_OFFSET;
    }

    public static Pose forAlliance(Alliance alliance, double redX, double redY, double redHeadingDeg) {
        double x = redX, y = redY, h = Math.toRadians(redHeadingDeg);
        if (alliance == Alliance.BLUE) {
            x = -x;
            y = -y;
            h += Math.PI;
        }
        return pedro(x, y, wrap(h));
    }

    public static double mouthX(Alliance alliance) {
        return alliance.hiveX();
    }

    public static double mouthY(int upSide) {
        return upSide * MOUTH_OUT;
    }

    public static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
