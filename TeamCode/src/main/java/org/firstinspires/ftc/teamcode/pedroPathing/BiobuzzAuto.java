package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.robot.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.Ball;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.Field;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.Hub;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.RobotConfig;

@Autonomous(name = "Later - Full Robot Auto", group = "2 Later (not built yet)", preselectTeleOp = "Later - Full Robot TeleOp")
public class BiobuzzAuto extends OpMode {
    enum Routine { FULL, PRELOAD_AND_PARK, PRELOAD_ONLY }

    enum Step { AIM_PRELOAD, WAIT_TIP, SWEEP, TO_FIRE, FIRE, PARK, DONE }

    // red poses in field inches (x toward the blue wall, y away from the audience); blue is mirrored
    static final double[] START = {-40, -61.5, 90};
    static final double SWEEP_X0 = -38, SWEEP_X1 = -4, SWEEP_Y = 33;
    static final double[] ZONE = {-58, 38};
    static final double[] AUDIENCE_POCKET = {-40, -52};
    // latest time a second cycle may start and still fire and park
    static final double LAST_CYCLE_START_S = 19;

    private Robot robot;
    private Alliance alliance = Alliance.RED;
    private Routine routine = Routine.FULL;
    private Step step;
    private long startMs, stepMs;
    private boolean firstLoop;
    private double heldHeading = Double.NaN;
    private boolean lastUp, lastDown;
    private int cycles;
    private int flipsSeen;
    private int spillSide;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, alliance, startPose());
        robot.inv.preload(Ball.POLLEN, Ball.POLLEN, Ball.POLLEN, Ball.POLLEN);
        robot.gramsInUpCell = RobotConfig.STAGED_NECTAR_G;
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void init_loop() {
        Alliance before = alliance;
        if (gamepad1.x) alliance = Alliance.BLUE;
        if (gamepad1.b) alliance = Alliance.RED;
        if (gamepad1.dpad_up && !lastUp) routine = Routine.values()[(routine.ordinal() + 2) % 3];
        if (gamepad1.dpad_down && !lastDown) routine = Routine.values()[(routine.ordinal() + 1) % 3];
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        if (alliance != before) {
            robot.alliance = alliance;
            robot.follower.setStartingPose(startPose());
        }
        robot.hive.update(Hub.nowMs());
        telemetry.addData("alliance", "%s   (x blue, b red)", alliance);
        telemetry.addData("routine", "%s   (dpad up/down)", routine);
        telemetry.addData("hive", "%s up-CELL on the %s side, %d tags", alliance,
                robot.hive.upSide(alliance) > 0 ? "far" : "audience", robot.hive.tagsSeen());
        telemetry.addLine("place the robot back against the audience wall at the start mark");
        telemetry.update();
    }

    @Override
    public void start() {
        startMs = Hub.nowMs();
        flipsSeen = robot.hive.flips(alliance);
        spillSide = redUpSide();
        go(Step.AIM_PRELOAD);
    }

    @Override
    public void loop() {
        robot.update();
        long now = robot.now();
        double t = (now - startMs) / 1000.0;
        double inStep = (now - stepMs) / 1000.0;
        boolean first = firstLoop;
        firstLoop = false;

        if (t > 29.3 && step != Step.DONE) go(Step.DONE);

        switch (step) {
            case AIM_PRELOAD:
                aimInPlace(first);
                robot.wantAim = robot.wantFire = true;
                if (robot.inv.held() == 0 || inStep > 8) go(Step.WAIT_TIP);
                break;
            case WAIT_TIP:
                robot.wantAim = robot.wantFire = false;
                if (first) spillSide = robot.hive.flips(alliance) == flipsSeen ? redUpSide() : -redUpSide();
                boolean tipped = robot.hive.flips(alliance) > flipsSeen;
                double banked = robot.gramsInUpCell + (cycles == 0 ? 0 : RobotConfig.PARTNER_AUTO_G);
                if (!tipped && inStep > 3.0 && banked >= RobotConfig.TIP_SURE_G) {
                    robot.hive.manualFlip(alliance);
                    tipped = true;
                }
                if (tipped) {
                    flipsSeen = robot.hive.flips(alliance);
                    if (routine == Routine.PRELOAD_ONLY) go(Step.DONE);
                    else if (routine == Routine.PRELOAD_AND_PARK || cycles >= 2 || t > LAST_CYCLE_START_S) startPark();
                    else {
                        robot.follower.setMaxPower(0.6);
                        robot.follower.followPath(sweep(spillSide), true);
                        go(Step.SWEEP);
                    }
                } else if (inStep > 4.5) {
                    if (routine == Routine.PRELOAD_ONLY) go(Step.DONE);
                    else startPark();
                }
                break;
            case SWEEP:
                robot.intakeMode = Intake.Mode.IN;
                if (!robot.follower.isBusy() || robot.inv.full() || inStep > 6) {
                    robot.follower.setMaxPower(1);
                    robot.follower.followPath(toFiringPose(), true);
                    go(Step.TO_FIRE);
                }
                break;
            case TO_FIRE:
                robot.intakeMode = robot.inv.full() ? Intake.Mode.OFF : Intake.Mode.IN;
                robot.wantAim = distanceTo(firingSpot()) < 24;
                if (!robot.follower.isBusy() || inStep > 5) go(Step.FIRE);
                break;
            case FIRE:
                robot.intakeMode = Intake.Mode.OFF;
                aimInPlace(first);
                robot.wantAim = robot.wantFire = robot.inv.held() > 0;
                if (robot.inv.held() == 0 || inStep > 6) {
                    cycles++;
                    if (routine == Routine.FULL && cycles < 2 && t < LAST_CYCLE_START_S) go(Step.WAIT_TIP);
                    else if (inZone()) go(Step.DONE);
                    else startPark();
                }
                break;
            case PARK:
                robot.wantAim = robot.wantFire = false;
                robot.intakeMode = Intake.Mode.OFF;
                if (!robot.follower.isBusy() || inStep > 6) go(Step.DONE);
                break;
            case DONE:
            default:
                robot.wantAim = robot.wantFire = false;
                robot.intakeMode = Intake.Mode.OFF;
                break;
        }

        telemetry.addData("auto", "%s  %.1f s  (step %.1f s, cycles %d)", step, t, inStep, cycles);
        robot.report(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.save();
        robot.hive.stop();
    }

    private void go(Step s) {
        step = s;
        stepMs = Hub.nowMs();
        firstLoop = true;
        heldHeading = Double.NaN;
    }

    private void startPark() {
        robot.follower.setMaxPower(1);
        robot.follower.followPath(toPose(ZONE, aimDegFrom(ZONE, 1)), true);
        go(Step.PARK);
    }

    private int redUpSide() {
        int up = robot.hive.upSide(alliance);
        return alliance == Alliance.RED ? up : -up;
    }

    private double[] firingSpot() {
        return redUpSide() > 0 ? ZONE : AUDIENCE_POCKET;
    }

    private boolean inZone() {
        return distanceTo(ZONE) < 6;
    }

    private double aimDegFrom(double[] spot, int redUp) {
        return Math.toDegrees(Math.atan2(redUp * Field.MOUTH_OUT - spot[1], -Field.HIVE_X - spot[0]));
    }

    private void aimInPlace(boolean first) {
        double target = robot.aim.headingTarget;
        if (first || Double.isNaN(heldHeading) || Math.abs(Math.toDegrees(Field.wrap(target - heldHeading))) > 1.5) {
            Pose p = robot.follower.getPose();
            robot.follower.holdPoint(new Pose(p.getX(), p.getY(), target));
            heldHeading = target;
        }
    }

    private Pose startPose() {
        return Field.forAlliance(alliance, START[0], START[1], START[2]);
    }

    private Pose at(double x, double y, double headingDeg) {
        return Field.forAlliance(alliance, x, y, headingDeg);
    }

    private double distanceTo(double[] xy) {
        Pose goal = at(xy[0], xy[1], 0);
        Pose p = robot.follower.getPose();
        return Math.hypot(goal.getX() - p.getX(), goal.getY() - p.getY());
    }

    private PathChain sweep(int redSpillSide) {
        double y = SWEEP_Y * redSpillSide;
        Pose here = robot.follower.getPose();
        Pose a = at(SWEEP_X0, y, 0), b = at(SWEEP_X1, y, 0);
        return robot.follower.pathBuilder()
                .addPath(new BezierCurve(here, at(-42, y + 12 * redSpillSide, 0), a))
                .setLinearHeadingInterpolation(here.getHeading(), a.getHeading())
                .addPath(new BezierLine(a, b))
                .setConstantHeadingInterpolation(a.getHeading())
                .build();
    }

    private PathChain toFiringPose() {
        double[] spot = firingSpot();
        return toPose(spot, aimDegFrom(spot, redUpSide()));
    }

    private PathChain toPose(double[] spot, double headingDeg) {
        Pose here = robot.follower.getPose();
        Pose end = at(spot[0], spot[1], headingDeg);
        double hereY = (alliance == Alliance.RED ? 1 : -1) * Field.y(here);
        double side = Math.signum(hereY == 0 ? -1 : hereY), goal = Math.signum(spot[1]);
        if (side == goal) {
            return robot.follower.pathBuilder()
                    .addPath(new BezierCurve(here, at(-50, (hereY + spot[1]) / 2, 0), end))
                    .setLinearHeadingInterpolation(here.getHeading(), end.getHeading())
                    .build();
        }
        return robot.follower.pathBuilder()
                .addPath(new BezierCurve(here, at(-60, 48 * side, 0), at(-62, 20 * goal, 0), end))
                .setLinearHeadingInterpolation(here.getHeading(), end.getHeading())
                .build();
    }
}
