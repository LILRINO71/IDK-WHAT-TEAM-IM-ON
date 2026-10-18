package org.firstinspires.ftc.teamcode.pedroPathing.robot;

import com.pedropathing.geometry.Pose;

public final class PoseStore {
    public static Pose pose;
    public static Alliance alliance = Alliance.RED;
    public static int redUp = Alliance.RED.startUpSide();
    public static int blueUp = Alliance.BLUE.startUpSide();
    public static Ball[] held = new Ball[0];
    public static double gramsInUpCell;
    public static long savedAtMs;

    private PoseStore() {
    }

    public static boolean fresh() {
        return pose != null && Hub.nowMs() - savedAtMs < 120000;
    }
}
