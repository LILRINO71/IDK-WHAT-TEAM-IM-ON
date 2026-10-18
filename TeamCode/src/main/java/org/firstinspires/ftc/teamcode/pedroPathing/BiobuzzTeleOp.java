package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.robot.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.Ball;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.Field;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.PoseStore;
import org.firstinspires.ftc.teamcode.pedroPathing.robot.Robot;

@TeleOp(name = "BIOBUZZ TeleOp", group = "BIOBUZZ")
public class BiobuzzTeleOp extends OpMode {
    private Robot robot;
    private boolean fieldCentric = true;
    private boolean wasFull;
    private final boolean[] last = new boolean[16];

    @Override
    public void init() {
        boolean fresh = PoseStore.fresh();
        Alliance alliance = fresh ? PoseStore.alliance : Alliance.RED;
        Pose start = fresh ? PoseStore.pose : Field.forAlliance(alliance, -40, -61.5, 90);
        robot = new Robot(hardwareMap, alliance, start);
        if (fresh) robot.restore();
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) robot.alliance = Alliance.BLUE;
        if (gamepad1.b) robot.alliance = Alliance.RED;
        telemetry.addData("alliance", "%s   (x blue, b red)", robot.alliance);
        telemetry.addData("start pose", PoseStore.fresh() ? "from auto" : "audience wall start");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        robot.update();

        double slow = gamepad1.left_stick_button ? 0.4 : 1.0;
        double fx = -gamepad1.left_stick_y * slow;
        double fy = -gamepad1.left_stick_x * slow;
        double turn = -gamepad1.right_stick_x * (slow < 1 ? 0.5 : 1.0);
        boolean aiming = gamepad1.right_bumper || gamepad1.left_bumper || gamepad2.right_bumper;
        if (aiming && Math.abs(gamepad1.right_stick_x) < 0.3) turn = robot.aimTurn();
        else robot.lock.reset();

        if (fieldCentric) {
            double a = robot.alliance.driverHeading();
            double h = robot.follower.getPose().getHeading();
            double gx = fx * Math.cos(a) - fy * Math.sin(a);
            double gy = fx * Math.sin(a) + fy * Math.cos(a);
            double rx = gx * Math.cos(h) + gy * Math.sin(h);
            double ry = -gx * Math.sin(h) + gy * Math.cos(h);
            robot.follower.setTeleOpDrive(rx, ry, turn, true);
        } else {
            robot.follower.setTeleOpDrive(fx, fy, turn, true);
        }

        robot.wantFire = gamepad1.right_bumper || gamepad2.right_bumper;
        robot.wantAim = aiming;
        if (gamepad1.right_trigger > 0.2 || gamepad2.right_trigger > 0.2) robot.intakeMode = Intake.Mode.IN;
        else if (gamepad1.left_trigger > 0.2 || gamepad2.left_trigger > 0.2) robot.intakeMode = Intake.Mode.EJECT;
        else robot.intakeMode = Intake.Mode.OFF;

        if (edge(0, gamepad1.b)) robot.jam.toggle();
        if (edge(1, gamepad1.y)) fieldCentric = !fieldCentric;
        if (edge(2, gamepad1.back)) {
            Pose p = robot.follower.getPose();
            robot.follower.setPose(new Pose(p.getX(), p.getY(), robot.alliance.driverHeading()));
        }
        if (edge(3, gamepad1.dpad_up)) robot.shooter.speedTrim += 0.01;
        if (edge(4, gamepad1.dpad_down)) robot.shooter.speedTrim -= 0.01;
        if (edge(5, gamepad1.dpad_right)) robot.shooter.pitchTrimDeg += 0.5;
        if (edge(6, gamepad1.dpad_left)) robot.shooter.pitchTrimDeg -= 0.5;
        if (edge(7, gamepad2.a)) robot.hive.manualFlip(robot.alliance);
        if (edge(8, gamepad2.dpad_up)) robot.inv.add(Ball.POLLEN);
        if (edge(9, gamepad2.dpad_left)) robot.inv.add(robot.alliance == Alliance.RED ? Ball.RED_NECTAR : Ball.BLUE_NECTAR);
        if (edge(10, gamepad2.dpad_down)) robot.inv.removeLast();
        if (edge(11, gamepad2.x)) robot.column.clearStuck();
        if (edge(12, gamepad2.b)) robot.jam.toggle();
        if (edge(13, gamepad2.y)) robot.gramsInUpCell = 0;

        if (robot.intake.takeRejectedOpponent()) gamepad1.rumbleBlips(1);
        robot.intake.takeRejectedFull();
        boolean full = robot.inv.full();
        if (full && !wasFull) gamepad1.rumble(250);
        wasFull = full;
        if (robot.tipSureJustNow()) {
            gamepad1.rumbleBlips(3);
            gamepad2.rumbleBlips(3);
        }

        telemetry.addData("drive", fieldCentric ? "field centric" : "robot centric");
        robot.report(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.save();
        robot.hive.stop();
    }

    private boolean edge(int i, boolean now) {
        boolean e = now && !last[i];
        last[i] = now;
        return e;
    }
}
