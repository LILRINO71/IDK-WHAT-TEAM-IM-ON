package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "Meet 0 TeleOp", group = "BIOBUZZ")
public class Meet0RollerTeleOp extends OpMode {
    // names in the Driver Hub robot configuration
    public static String INTAKE = "backIntake";
    public static String ROLLER_A = "rollerA";
    public static String ROLLER_B = "rollerB";
    public static String GATE = "gate";
    public static String FEEDER = "feed";

    // flip any of these if that mechanism runs backwards
    public static boolean INTAKE_REVERSED = true;
    public static boolean ROLLER_A_REVERSED = false;
    public static boolean ROLLER_B_REVERSED = true;
    public static boolean FEEDER_REVERSED = false;

    public static double TICKS_PER_REV = 28;
    public static double ROLLER_RPM = 3300;
    public static double RPM_STEP = 50;
    // extra roller speed while LB is held for a NECTAR, tune on the field
    public static double NECTAR_BOOST = 0.04;
    public static double READY_BAND = 0.03;
    public static double ROLLER_P = 12;
    public static double ROLLER_I = 0.5;
    public static double ROLLER_D = 0;
    public static double ROLLER_F = 13;

    // servo positions with the gate bar down across the floor and swung up
    public static double GATE_CLOSED = 0.15;
    public static double GATE_OPEN = 0.48;

    public static double INTAKE_POWER = 1.0;
    public static double FEED_POWER = 0.7;
    public static double EJECT_POWER = -0.8;
    // zip-tie feeder: flicks balls on the floor forward to the roller
    public static double FEEDER_POWER = 1.0;
    public static double FEEDER_EJECT_POWER = -0.6;

    private Follower follower;
    private DcMotorEx intake, rollerA, rollerB;
    private Servo gate;
    private CRServo feeder;
    private boolean rollerOn;
    private double rpm;
    private boolean lastA, lastUp, lastDown, lastBack;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        intake = motor(INTAKE, INTAKE_REVERSED);
        rollerA = motor(ROLLER_A, ROLLER_A_REVERSED);
        rollerB = motor(ROLLER_B, ROLLER_B_REVERSED);
        for (DcMotorEx m : new DcMotorEx[] {rollerA, rollerB}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setVelocityPIDFCoefficients(ROLLER_P, ROLLER_I, ROLLER_D, ROLLER_F);
        }
        gate = hardwareMap.get(Servo.class, GATE);
        gate.setPosition(GATE_CLOSED);
        feeder = hardwareMap.get(CRServo.class, FEEDER);
        feeder.setDirection(FEEDER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        rpm = ROLLER_RPM;
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        follower.update();
        double slow = gamepad1.left_stick_button ? 0.4 : 1.0;
        follower.setTeleOpDrive(-gamepad1.left_stick_y * slow, -gamepad1.left_stick_x * slow,
                -gamepad1.right_stick_x * (slow < 1 ? 0.5 : 1.0), true);

        if (gamepad1.a && !lastA) rollerOn = !rollerOn;
        if (gamepad1.dpad_up && !lastUp) rpm += RPM_STEP;
        if (gamepad1.dpad_down && !lastDown) rpm -= RPM_STEP;
        if (gamepad1.back && !lastBack) rpm = ROLLER_RPM;
        lastA = gamepad1.a;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastBack = gamepad1.back;

        boolean shoot = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean spin = rollerOn || shoot;
        boolean nectar = gamepad1.left_bumper || gamepad2.left_bumper;
        double target = nectar ? rpm * (1 + NECTAR_BOOST) : rpm;
        double tps = spin ? target * TICKS_PER_REV / 60 : 0;
        rollerA.setVelocity(tps);
        rollerB.setVelocity(tps);
        double rpmA = rollerA.getVelocity() * 60 / TICKS_PER_REV;
        double rpmB = rollerB.getVelocity() * 60 / TICKS_PER_REV;
        boolean ready = spin && Math.abs(rpmA - target) < target * READY_BAND && Math.abs(rpmB - target) < target * READY_BAND;

        boolean feeding = shoot && ready;
        gate.setPosition(feeding ? GATE_OPEN : GATE_CLOSED);
        boolean eject = gamepad1.left_trigger > 0.2 || gamepad2.left_trigger > 0.2;
        if (feeding) intake.setPower(FEED_POWER);
        else if (gamepad1.right_trigger > 0.2 || gamepad2.right_trigger > 0.2) intake.setPower(INTAKE_POWER);
        else if (eject) intake.setPower(EJECT_POWER);
        else intake.setPower(0);
        feeder.setPower(feeding ? FEEDER_POWER : (eject ? FEEDER_EJECT_POWER : 0));

        if (shoot && !ready) gamepad1.rumble(50);
        telemetry.addData("roller", "%s  target %.0f rpm%s  A %.0f  B %.0f", ready ? "READY" : (spin ? "spinning up" : "off"), target, nectar ? " (NECTAR)" : "", rpmA, rpmB);
        telemetry.addData("gate", feeding ? "open" : "closed");
        telemetry.addData("controls", "A roller on/off, hold RB to shoot, hold LB for a NECTAR, RT intake, LT eject, dpad up/down rpm, back reset rpm");
        telemetry.update();
    }

    @Override
    public void stop() {
        rollerA.setVelocity(0);
        rollerB.setVelocity(0);
        intake.setPower(0);
        feeder.setPower(0);
    }

    private DcMotorEx motor(String name, boolean reversed) {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
        m.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        return m;
    }
}
