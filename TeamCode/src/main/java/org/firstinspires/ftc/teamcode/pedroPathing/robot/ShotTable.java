package org.firstinspires.ftc.teamcode.pedroPathing.robot;

public final class ShotTable {
    static final double[] RANGE_IN = {12.5, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70};
    static final double[] BEARING_DEG = {0, 20, 40, 60, 75};
    static final double[][] POLLEN_PITCH = {
            {83.5, 81.8, 78.0, 75.0, 72.0, 69.0, 67.0, 65.0, 65.0, 65.0, 65.0, 65.0, 65.0},
            {83.3, 81.5, 78.0, 75.0, 72.0, 70.0, 68.0, 65.0, 65.0, 65.0, 65.0, 65.0, 65.0},
            {84.3, 82.0, 79.0, 76.0, 74.0, 70.0, 68.0, 66.0, 64.0, 61.0, 61.0, 61.0, 61.0},
            {86.0, 82.8, 81.0, 78.0, 75.0, 72.0, 69.0, 67.0, 65.0, 65.0, 65.0, 65.0, 65.0},
            {87.0, 87.0, 83.3, 80.0, 76.0, 73.0, 71.0, 68.3, 66.3, 66.3, 66.3, 66.3, 66.3}
    };
    static final double[][] POLLEN_SPEED = {
            {5.022, 5.022, 4.948, 4.973, 5.047, 5.124, 5.200, 5.304, 5.304, 5.304, 5.304, 5.304, 5.304},
            {4.998, 4.973, 4.948, 4.973, 5.022, 5.098, 5.200, 5.304, 5.304, 5.304, 5.304, 5.304, 5.304},
            {5.124, 4.998, 4.973, 4.973, 5.047, 5.098, 5.174, 5.278, 5.383, 5.491, 5.491, 5.491, 5.491},
            {5.798, 5.251, 5.098, 5.047, 5.047, 5.098, 5.174, 5.251, 5.356, 5.463, 5.463, 5.463, 5.463},
            {6.433, 6.433, 5.383, 5.174, 5.174, 5.174, 5.251, 5.304, 5.410, 5.410, 5.410, 5.410, 5.410}
    };
    static final double[][] NECTAR_PITCH = {
            {83.3, 81.8, 78.3, 75.0, 72.0, 71.0, 65.0, 63.0, 63.0, 63.0, 63.0, 63.0, 63.0},
            {83.5, 81.8, 78.3, 75.0, 73.0, 70.0, 67.0, 65.0, 65.0, 65.0, 65.0, 65.0, 65.0},
            {84.0, 82.5, 79.3, 76.0, 73.0, 70.0, 68.0, 66.0, 64.0, 62.0, 62.0, 62.0, 62.0},
            {86.5, 85.0, 81.5, 78.0, 75.0, 72.0, 70.0, 65.0, 63.0, 61.0, 61.0, 61.0, 61.0},
            {87.5, 87.5, 83.0, 79.8, 76.8, 74.8, 71.3, 69.3, 68.0, 68.0, 68.0, 68.0, 68.0}
    };
    static final double[][] NECTAR_SPEED = {
            {4.973, 4.998, 4.973, 4.973, 5.047, 5.148, 5.251, 5.356, 5.356, 5.356, 5.356, 5.356, 5.356},
            {5.022, 4.973, 4.948, 4.973, 4.998, 5.098, 5.200, 5.304, 5.304, 5.304, 5.304, 5.304, 5.304},
            {5.148, 5.047, 4.973, 4.973, 4.998, 5.098, 5.174, 5.278, 5.383, 5.491, 5.491, 5.491, 5.491},
            {6.184, 5.546, 5.173, 5.073, 5.073, 5.098, 5.174, 5.278, 5.383, 5.491, 5.491, 5.491, 5.491},
            {7.102, 7.102, 5.383, 5.225, 5.174, 5.251, 5.278, 5.356, 5.463, 5.463, 5.463, 5.463, 5.463}
    };

    private ShotTable() {
    }

    public static double pitchDeg(double rangeIn, double bearingDeg, Ball ball) {
        return lookup(ball.isNectar() ? NECTAR_PITCH : POLLEN_PITCH, rangeIn, bearingDeg);
    }

    public static double speedMps(double rangeIn, double bearingDeg, Ball ball) {
        return lookup(ball.isNectar() ? NECTAR_SPEED : POLLEN_SPEED, rangeIn, bearingDeg);
    }

    public static boolean inRange(double rangeIn, double bearingDeg) {
        return rangeIn >= 15 && rangeIn <= 62 && Math.abs(bearingDeg) <= RobotConfig.MAX_BEARING_DEG;
    }

    public static double flightSeconds(double rangeIn, double pitchDeg, double speedMps) {
        double horizontal = speedMps * Math.cos(Math.toRadians(pitchDeg));
        return horizontal <= 0.05 ? 0 : rangeIn * 0.0254 / horizontal;
    }

    private static double lookup(double[][] grid, double rangeIn, double bearingDeg) {
        double b = Math.min(Math.abs(bearingDeg), BEARING_DEG[BEARING_DEG.length - 1]);
        int bi = index(BEARING_DEG, b), ri = index(RANGE_IN, rangeIn);
        double bt = frac(BEARING_DEG, bi, b), rt = frac(RANGE_IN, ri, rangeIn);
        double a = grid[bi][ri] + (grid[bi][ri + 1] - grid[bi][ri]) * rt;
        double c = grid[bi + 1][ri] + (grid[bi + 1][ri + 1] - grid[bi + 1][ri]) * rt;
        return a + (c - a) * bt;
    }

    private static int index(double[] xs, double x) {
        for (int i = 0; i < xs.length - 2; i++) if (x < xs[i + 1]) return i;
        return xs.length - 2;
    }

    private static double frac(double[] xs, int i, double x) {
        double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
        return Math.max(0, Math.min(1, t));
    }
}
