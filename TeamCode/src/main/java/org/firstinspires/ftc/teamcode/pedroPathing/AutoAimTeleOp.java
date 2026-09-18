package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp(name = "Auto Aim TeleOp")
public class AutoAimTeleOp extends LinearOpMode {

    static final boolean FIELD_CENTRIC = true;

    // raise this if strafing is slower than driving forward
    static final double STRAFE_FIX = 1.1;

    // make this negative if the robot turns away from the cell
    static final double AIM_KP = 0.02;
    // lowest turn power that still moves the robot
    static final double AIM_MIN_POWER = 0.05;
    static final double AIM_TOLERANCE = 1.0;
    // degrees, change this if shots land left or right of the cell
    static final double AIM_OFFSET = 0;

    static final String[] CELLS = {"red scoring", "red audience", "blue audience", "blue scoring"};

    @Override
    public void runOpMode() {
        // names have to match the robot config on the driver hub
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // flip any wheel that spins backwards when you push the left stick forward
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, // which way the control hub logo is facing
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD))); // which way the control hub usb ports are facing
        imu.resetYaw();

        // apriltag pipeline number from the limelight web page (36h11, tag size 82.55 mm, full 3d on)
        limelight.pipelineSwitch(0);
        limelight.start();

        boolean red = true;
        while (opModeInInit()) {
            if (gamepad1.b) red = true;
            if (gamepad1.x) red = false;
            telemetry.addData("alliance", red ? "red" : "blue");
            telemetry.addLine("b = red, x = blue");
            telemetry.update();
        }

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double[] area = new double[4];
            double[] tx = new double[4];
            double[] distance = new double[4];
            int[] seen = new int[4];

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid() && result.getStaleness() < 100) {
                int first = red ? 30 : 38;
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    int id = tag.getFiducialId();
                    if (id < first || id > first + 7) continue;
                    int cell = (id - 30) / 4;
                    Position p = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
                    area[cell] += tag.getTargetArea();
                    tx[cell] += tag.getTargetXDegrees();
                    distance[cell] += Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                    seen[cell]++;
                }
            }

            int up = -1;
            for (int cell = 0; cell < 4; cell++) {
                if (seen[cell] > 0 && (up == -1 || area[cell] > area[up])) up = cell;
            }

            if (up != -1) {
                double error = tx[up] / seen[up] - AIM_OFFSET;
                if (gamepad1.left_bumper) {
                    double aim = AIM_KP * error;
                    turn = Math.abs(error) > AIM_TOLERANCE ? aim + Math.copySign(AIM_MIN_POWER, aim) : 0;
                }
                telemetry.addData("up cell", "%s  %d tags  %.1f deg  %.1f in",
                        CELLS[up], seen[up], error, distance[up] / seen[up]);
            } else {
                telemetry.addData("up cell", "not in view");
            }

            double heading = FIELD_CENTRIC ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : 0;
            double x = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
            double y = strafe * Math.sin(-heading) + forward * Math.cos(-heading);
            x *= STRAFE_FIX;

            double max = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(turn), 1);
            leftFront.setPower((y + x + turn) / max);
            leftRear.setPower((y - x + turn) / max);
            rightFront.setPower((y - x - turn) / max);
            rightRear.setPower((y + x - turn) / max);

            telemetry.addData("heading", "%.1f", Math.toDegrees(heading));
            telemetry.update();
        }

        limelight.stop();
    }
}
