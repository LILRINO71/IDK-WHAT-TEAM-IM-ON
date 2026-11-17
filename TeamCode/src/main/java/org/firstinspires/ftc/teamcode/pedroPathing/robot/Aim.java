package org.firstinspires.ftc.teamcode.pedroPathing.robot;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public class Aim {
    public double rangeIn, bearingDeg, headingTarget, headingErrorDeg;
    public double pitchDeg, speedMps, flightS;
    public double targetX, targetY;
    public double robotSpeed;
    public boolean shootable;

    public void solve(Pose pose, Vector velocity, Alliance alliance, int upSide, Ball ball) {
        double h = pose.getHeading();
        double rx = Field.x(pose) + RobotConfig.SHOOTER_FORWARD_IN * Math.cos(h);
        double ry = Field.y(pose) + RobotConfig.SHOOTER_FORWARD_IN * Math.sin(h);
        double vx = velocity == null ? 0 : velocity.getXComponent();
        double vy = velocity == null ? 0 : velocity.getYComponent();
        robotSpeed = Math.hypot(vx, vy);
        double mx = Field.mouthX(alliance), my = Field.mouthY(upSide);
        Ball b = ball == Ball.NONE ? Ball.POLLEN : ball;
        targetX = mx;
        targetY = my;
        for (int i = 0; i < 4; i++) {
            double dx = targetX - rx, dy = targetY - ry;
            rangeIn = Math.hypot(dx, dy);
            bearingDeg = bearing(rx - mx, ry - my, upSide);
            pitchDeg = ShotTable.pitchDeg(rangeIn, bearingDeg, b);
            speedMps = ShotTable.speedMps(rangeIn, bearingDeg, b);
            flightS = ShotTable.flightSeconds(rangeIn, pitchDeg, speedMps);
            targetX = mx - vx * flightS * RobotConfig.LEAD_SCALE;
            targetY = my - vy * flightS * RobotConfig.LEAD_SCALE;
        }
        headingTarget = Math.atan2(targetY - ry, targetX - rx);
        headingErrorDeg = Math.toDegrees(Field.wrap(headingTarget - h));
        shootable = ShotTable.inRange(rangeIn, bearingDeg) && robotSpeed <= RobotConfig.MAX_SHOOT_SPEED_IN_S;
    }

    public boolean aimed() {
        return shootable && Math.abs(headingErrorDeg) <= RobotConfig.AIM_TOLERANCE_DEG;
    }

    private static double bearing(double ox, double oy, int upSide) {
        double along = oy * upSide;
        return Math.toDegrees(Math.atan2(Math.abs(ox), along));
    }
}
