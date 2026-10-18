package org.firstinspires.ftc.teamcode.pedroPathing.robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class HiveTracker {
    private final Limelight3A limelight;
    private final int[] upSide = {Alliance.RED.startUpSide(), Alliance.BLUE.startUpSide()};
    private final int[] candidate = {0, 0};
    private final long[] candidateSinceMs = {-1, -1};
    private final int[] flips = {0, 0};
    private long lastReadMs;
    private long lastSeenMs = -100000;
    private int tagsSeen;

    public HiveTracker(HardwareMap hw) {
        Limelight3A ll = null;
        try {
            ll = hw.get(Limelight3A.class, RobotConfig.LIMELIGHT);
            ll.pipelineSwitch(RobotConfig.LIMELIGHT_PIPELINE);
            ll.start();
        } catch (Exception e) {
            ll = null;
        }
        limelight = ll;
    }

    public void set(Alliance a, int side) {
        upSide[a.ordinal()] = side;
        candidateSinceMs[a.ordinal()] = -1;
    }

    public void manualFlip(Alliance a) {
        set(a, -upSide(a));
        flips[a.ordinal()]++;
    }

    public int upSide(Alliance a) {
        return upSide[a.ordinal()];
    }

    public int flips(Alliance a) {
        return flips[a.ordinal()];
    }

    public boolean seesTags(long now) {
        return now - lastSeenMs < 500;
    }

    public int tagsSeen() {
        return tagsSeen;
    }

    public boolean connected() {
        return limelight != null && limelight.isConnected();
    }

    public void update(long now) {
        if (limelight == null || now - lastReadMs < 50) return;
        lastReadMs = now;
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid() || r.getStaleness() > 150) return;
        double[] best = {Double.NaN, Double.NaN};
        int[] bestSide = {0, 0};
        tagsSeen = 0;
        for (LLResultTypes.FiducialResult tag : r.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (id < 30 || id > 45) continue;
            Alliance a = id <= 37 ? Alliance.RED : Alliance.BLUE;
            int side = (id <= 33 || id >= 42) ? 1 : -1;
            double h = heightIn(tag);
            if (Double.isNaN(h)) continue;
            tagsSeen++;
            int i = a.ordinal();
            if (Double.isNaN(best[i]) || h > best[i]) {
                best[i] = h;
                bestSide[i] = side;
            }
        }
        if (tagsSeen > 0) lastSeenMs = now;
        for (int i = 0; i < 2; i++) {
            if (Double.isNaN(best[i])) continue;
            int vote = best[i] >= RobotConfig.UP_CELL_TAG_HEIGHT_IN ? bestSide[i] : -bestSide[i];
            if (vote == upSide[i]) {
                candidateSinceMs[i] = -1;
                continue;
            }
            if (candidateSinceMs[i] < 0 || candidate[i] != vote) {
                candidate[i] = vote;
                candidateSinceMs[i] = now;
            } else if (now - candidateSinceMs[i] >= RobotConfig.HIVE_FLIP_CONFIRM_MS) {
                upSide[i] = vote;
                flips[i]++;
                candidateSinceMs[i] = -1;
            }
        }
    }

    private static double heightIn(LLResultTypes.FiducialResult tag) {
        Position p = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
        if (p.x == 0 && p.y == 0 && p.z == 0) return Double.NaN;
        double pitch = Math.toRadians(RobotConfig.CAMERA_PITCH_DEG);
        return RobotConfig.CAMERA_HEIGHT_IN - p.y * Math.cos(pitch) + p.z * Math.sin(pitch);
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }
}
