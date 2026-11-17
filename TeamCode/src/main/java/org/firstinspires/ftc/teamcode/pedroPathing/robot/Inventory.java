package org.firstinspires.ftc.teamcode.pedroPathing.robot;

import java.util.ArrayDeque;
import java.util.Iterator;

public class Inventory {
    private static final long PENDING_EXPIRE_MS = 1500;

    private final ArrayDeque<Ball> held = new ArrayDeque<>();
    private final ArrayDeque<Ball> pending = new ArrayDeque<>();
    private final ArrayDeque<Long> pendingAt = new ArrayDeque<>();
    private int fired;
    private double firedGrams;

    public void preload(Ball... balls) {
        held.clear();
        pending.clear();
        pendingAt.clear();
        for (Ball b : balls) if (b != Ball.NONE && held.size() < RobotConfig.MAX_HELD) held.addLast(b);
    }

    public void accepted(Ball b, long nowMs) {
        pending.addLast(b);
        pendingAt.addLast(nowMs);
    }

    public void enteredColumn() {
        Ball b = pending.isEmpty() ? Ball.POLLEN : pending.pollFirst();
        if (!pendingAt.isEmpty()) pendingAt.pollFirst();
        if (held.size() < RobotConfig.MAX_HELD) held.addLast(b);
    }

    public void update(long nowMs) {
        while (!pendingAt.isEmpty() && nowMs - pendingAt.peekFirst() > PENDING_EXPIRE_MS) {
            pendingAt.pollFirst();
            pending.pollFirst();
        }
    }

    public Ball fired() {
        Ball b = held.isEmpty() ? Ball.POLLEN : held.pollFirst();
        fired++;
        firedGrams += b.grams();
        return b;
    }

    public void add(Ball b) {
        if (held.size() < RobotConfig.MAX_HELD) held.addLast(b);
    }

    public void removeLast() {
        if (!held.isEmpty()) held.pollLast();
    }

    public Ball next() {
        return held.isEmpty() ? Ball.NONE : held.peekFirst();
    }

    public int held() {
        return held.size();
    }

    public int total() {
        return held.size() + pending.size();
    }

    public boolean full() {
        return total() >= RobotConfig.MAX_HELD;
    }

    public int firedCount() {
        return fired;
    }

    public double firedGrams() {
        return firedGrams;
    }

    public double heldGrams() {
        double g = 0;
        for (Ball b : held) g += b.grams();
        return g;
    }

    public Ball[] snapshot() {
        Ball[] out = new Ball[held.size()];
        Iterator<Ball> it = held.iterator();
        for (int i = 0; i < out.length; i++) out[i] = it.next();
        return out;
    }

    public String describe() {
        StringBuilder s = new StringBuilder();
        for (Ball b : held) s.append(b.shortName()).append(' ');
        if (!pending.isEmpty()) s.append("+").append(pending.size()).append(" coming");
        return s.length() == 0 ? "empty" : s.toString().trim();
    }
}
