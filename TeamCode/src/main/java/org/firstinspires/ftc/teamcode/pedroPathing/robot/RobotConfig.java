package org.firstinspires.ftc.teamcode.pedroPathing.robot;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConfig {
    // names in the Driver Hub robot configuration
    public static final String FRONT_INTAKE = "frontIntake";
    public static final String BACK_INTAKE = "backIntake";
    public static final String COLUMN = "column";
    public static final String FLYWHEEL = "flywheel";
    public static final String PITCH_SERVO = "pitch";
    public static final String STOPPER_SERVO = "stopper";
    public static final String LEFT_JAM_SERVO = "leftJam";
    public static final String RIGHT_JAM_SERVO = "rightJam";
    public static final String FRONT_COLOR = "frontColor";
    public static final String BACK_COLOR = "backColor";
    public static final String COLUMN_IN = "columnIn";
    public static final String COLUMN_TOP = "columnTop";
    public static final String LIMELIGHT = "limelight";

    // flip any of these if that mechanism runs backwards
    public static boolean FRONT_INTAKE_REVERSED = false;
    public static boolean BACK_INTAKE_REVERSED = true;
    public static boolean COLUMN_REVERSED = false;
    public static boolean FLYWHEEL_REVERSED = false;
    public static boolean BEAM_BROKEN_IS_LOW = true;

    public static double FLYWHEEL_TICKS_PER_MOTOR_REV = 28;
    public static double FLYWHEEL_WHEEL_PER_MOTOR = 0.5;
    public static double FLYWHEEL_WHEEL_DIAMETER_M = 0.096;
    // tune with Shooter Tuner until shots land where the table says
    public static double EXIT_EFFICIENCY = 0.90;
    public static double FLY_KS = 0.04;
    public static double FLY_KV = 0.00036;
    public static double FLY_KP = 0.0009;
    public static double FLY_KI = 0.0006;
    public static double NOMINAL_VOLTAGE = 12.0;
    public static double READY_BAND = 0.03;
    public static double LEAVE_BAND = 0.06;
    public static double READY_HOLD_MS = 80;
    public static double BOOST_BELOW = 0.12;
    public static double SHOT_DIP = 0.045;
    public static double IDLE_FRACTION = 0.6;

    public static double PITCH_MIN_DEG = 60;
    public static double PITCH_MAX_DEG = 87;
    // servo positions with the shooter pod at PITCH_MIN_DEG and PITCH_MAX_DEG
    public static double PITCH_SERVO_AT_MIN = 0.20;
    public static double PITCH_SERVO_AT_MAX = 0.47;
    public static double PITCH_SETTLE_MS_PER_DEG = 6;

    public static double STOPPER_CLOSED = 0.15;
    public static double STOPPER_OPEN = 0.55;
    public static double STOPPER_MOVE_MS = 70;
    public static double FEED_TIMEOUT_MS = 450;

    public static double JAM_UP = 0.10;
    public static double JAM_DOWN = 0.75;

    public static double INTAKE_POWER = 1.0;
    public static double INTAKE_SLOW_POWER = 0.35;
    public static double INTAKE_EJECT_POWER = -0.8;
    public static double EJECT_MS = 450;
    public static double MERGE_HOLD_MS = 650;
    public static double COLUMN_INDEX_POWER = 0.55;
    public static double COLUMN_FEED_POWER = 1.0;
    public static double STALL_AMPS = 5.5;
    public static double STALL_MS = 180;
    public static double UNJAM_MS = 220;
    public static double UNJAM_POWER = -0.6;
    public static double UNJAM_COOLDOWN_MS = 500;
    public static double COLUMN_STUCK_MS = 2000;

    // read these off the Hardware Check opmode with each ball held at the sensor
    public static double BALL_PRESENT_MM = 70;
    public static double MIN_SATURATION = 0.30;
    public static double POLLEN_HUE_MIN = 35;
    public static double POLLEN_HUE_MAX = 85;
    public static double BLUE_HUE_MIN = 185;
    public static double BLUE_HUE_MAX = 260;
    public static double RED_HUE_BELOW = 25;
    public static double RED_HUE_ABOVE = 330;
    public static int COLOR_SAMPLES = 2;

    public static int LIMELIGHT_PIPELINE = 0;
    // lens height off the tiles and tilt up from level
    public static double CAMERA_HEIGHT_IN = 15.5;
    public static double CAMERA_PITCH_DEG = 20;
    public static double UP_CELL_TAG_HEIGHT_IN = 50;
    public static double HIVE_FLIP_CONFIRM_MS = 250;

    // how far ahead of the robot centre the ball leaves the shooter
    public static double SHOOTER_FORWARD_IN = 0.0;
    public static double HEADING_KP = 2.2;
    public static double HEADING_KD = 0.12;
    public static double HEADING_KS = 0.06;
    public static double AIM_TOLERANCE_DEG = 2.0;
    public static double MAX_BEARING_DEG = 78;
    public static double LEAD_SCALE = 1.0;
    public static double MAX_SHOOT_SPEED_IN_S = 30;

    public static double POLLEN_G = 24.9;
    public static double NECTAR_G = 41.3;
    public static double TIP_SURE_G = 199;
    public static double TIP_MAYBE_G = 174;
    public static double STAGED_NECTAR_G = 123.9;
    // grams your partner banks in the far CELL during auto (0 if unsure)
    public static double PARTNER_AUTO_G = 0;
    public static int MAX_HELD = 4;
}
