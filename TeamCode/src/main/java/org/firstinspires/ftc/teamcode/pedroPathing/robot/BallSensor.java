package org.firstinspires.ftc.teamcode.pedroPathing.robot;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BallSensor {
    private final RevColorSensorV3 sensor;
    private boolean present;
    private Ball ball = Ball.NONE;
    private double distMm = 999, hue, sat;
    private double sumR, sumG, sumB;
    private int good;
    private int arrivals;

    public BallSensor(HardwareMap hw, String name) {
        sensor = hw.get(RevColorSensorV3.class, name);
        sensor.setGain(8);
    }

    public void update() {
        distMm = sensor.getDistance(DistanceUnit.MM);
        if (Double.isNaN(distMm) || distMm > RobotConfig.BALL_PRESENT_MM) {
            present = false;
            ball = Ball.NONE;
            sumR = sumG = sumB = 0;
            good = 0;
            return;
        }
        present = true;
        NormalizedRGBA c = sensor.getNormalizedColors();
        double[] hs = hueSat(c.red, c.green, c.blue);
        hue = hs[0];
        sat = hs[1];
        if (sat >= RobotConfig.MIN_SATURATION) {
            sumR += c.red;
            sumG += c.green;
            sumB += c.blue;
            good++;
        }
        if (good >= RobotConfig.COLOR_SAMPLES && ball == Ball.NONE) {
            double[] avg = hueSat(sumR / good, sumG / good, sumB / good);
            ball = classify(avg[0], avg[1]);
            if (ball != Ball.NONE) arrivals++;
        }
    }

    public static Ball classify(double hue, double sat) {
        if (sat < RobotConfig.MIN_SATURATION) return Ball.NONE;
        if (hue >= RobotConfig.POLLEN_HUE_MIN && hue <= RobotConfig.POLLEN_HUE_MAX) return Ball.POLLEN;
        if (hue >= RobotConfig.BLUE_HUE_MIN && hue <= RobotConfig.BLUE_HUE_MAX) return Ball.BLUE_NECTAR;
        if (hue < RobotConfig.RED_HUE_BELOW || hue > RobotConfig.RED_HUE_ABOVE) return Ball.RED_NECTAR;
        return Ball.NONE;
    }

    public static double[] hueSat(double r, double g, double b) {
        double max = Math.max(r, Math.max(g, b)), min = Math.min(r, Math.min(g, b));
        double d = max - min;
        double h;
        if (d <= 1e-9) h = 0;
        else if (max == r) h = 60 * (((g - b) / d) % 6);
        else if (max == g) h = 60 * ((b - r) / d + 2);
        else h = 60 * ((r - g) / d + 4);
        if (h < 0) h += 360;
        double s = max <= 1e-9 ? 0 : d / max;
        return new double[]{h, s};
    }

    public boolean present() {
        return present;
    }

    public int arrivals() {
        return arrivals;
    }

    public Ball ball() {
        return ball;
    }

    public double distMm() {
        return distMm;
    }

    public double hue() {
        return hue;
    }

    public double sat() {
        return sat;
    }
}
