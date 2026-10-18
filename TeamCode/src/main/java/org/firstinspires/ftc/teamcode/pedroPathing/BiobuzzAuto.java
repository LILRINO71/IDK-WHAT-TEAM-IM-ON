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

@Autonomous(name = "BIOBUZZ Auto", group = "BIOBUZZ", preselectTeleOp = "BIOBUZZ TeleOp")
public class BiobuzzAuto extends OpMode {
    enum Routine { FULL, PRELOAD_AND_PARK, PRELOAD_ONLY }

    enum Step { AIM_PRELOAD, WAIT_TIP, SWEEP, TO_ZONE, FIRE_ZONE, PARK, DONE }

    // red poses in field inches (x toward the blue wall, y away from the audience); blue is mirrored
    static final double[] START = {-40, -61.5, 90};
    static final double[] SWEEP_A = {-38, -33};
    static final double[] SWEEP_B = {-4, -33};
    static final double[] ZONE = {-58, 38};

    private Robot robot;
    private Alliance alliance = Alliance.RED;
    private Routine routine = Routine.FULL;
    private Step step;
    private long startMs, stepMs;
    private boolean firstLoop;
    private double heldHeading = Double.NaN;
    private boolean lastUp, lastDown;

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
                boolean flipped = robot.hive.flips(alliance) > 0;
                if (!flipped && inStep > 3.5 && robot.gramsInUpCell >= RobotConfig.TIP_SURE_G) {
                    robot.hive.manualFlip(alliance);
                    flipped = true;
                }
                if (flipped || inStep > 4.5) {
                    if (routine == Routine.PRELOAD_ONLY) go(Step.DONE);
                    else if (routine == Routine.PRELOAD_AND_PARK) {
                        robot.follower.followPath(toZone(), true);
                        go(Step.PARK);
                    } else {
                        robot.follower.setMaxPower(0.6);
                        robot.follower.followPath(sweep(), true);
                        go(Step.SWEEP);
                    }
                }
                break;
            case SWEEP:
                robot.intakeMode = Intake.Mode.IN;
                if (!robot.follower.isBusy() || robot.inv.full() || inStep > 6) {
                    robot.follower.setMaxPower(1);
                    robot.follower.followPath(toZone(), true);
                    go(Step.TO_ZONE);
                }
                break;
            case TO_ZONE:
                robot.intakeMode = robot.inv.full() ? Intake.Mode.OFF : Intake.Mode.IN;
                robot.wantAim = distanceTo(ZONE) < 24;
                if (!robot.follower.isBusy() || inStep > 5) go(Step.FIRE_ZONE);
                break;
            case FIRE_ZONE:
                robot.intakeMode = Intake.Mode.OFF;
                aimInPlace(first);
                robot.wantAim = robot.wantFire = robot.inv.held() > 0;
                if (robot.inv.held() == 0 || inStep > 7) go(Step.DONE);
                break;
            case PARK:
                if (!robot.follower.isBusy() || inStep > 6) go(Step.DONE);
                break;
            case DONE:
            default:
                robot.wantAim = robot.wantFire = false;
                robot.intakeMode = Intake.Mode.OFF;
                break;
        }

        telemetry.addData("auto", "%s  %.1f s  (step %.1f s)", step, t, inStep);
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

    private PathChain sweep() {
        Pose here = robot.follower.getPose();
        Pose a = at(SWEEP_A[0], SWEEP_A[1], 0), b = at(SWEEP_B[0], SWEEP_B[1], 0);
        return robot.follower.pathBuilder()
                .addPath(new BezierCurve(here, at(-40, -45, 0), a))
                .setLinearHeadingInterpolation(here.getHeading(), a.getHeading())
                .addPath(new BezierLine(a, b))
                .setConstantHeadingInterpolation(a.getHeading())
                .build();
    }

    private PathChain toZone() {
        Pose here = robot.follower.getPose();
        double aimDeg = Math.toDegrees(Math.atan2(Field.MOUTH_OUT - ZONE[1], -Field.HIVE_X - ZONE[0]));
        Pose end = at(ZONE[0], ZONE[1], aimDeg);
        return robot.follower.pathBuilder()
                .addPath(new BezierCurve(here, at(-60, -48, 0), at(-62, 20, 0), end))
                .setLinearHeadingInterpolation(here.getHeading(), end.getHeading())
                .build();
    }
}
