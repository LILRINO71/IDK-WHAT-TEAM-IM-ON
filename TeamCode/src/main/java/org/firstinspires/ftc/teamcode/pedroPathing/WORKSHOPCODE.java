package org.firstinspires.ftc.teamcode.pedroPathing;

// ===== IMPORTS =====
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Basic Claw Test")
public class WORKSHOPCODE extends LinearOpMode {

    // =================================================
    // SERVOS
    // =================================================

    // Speed servo that opens and closes the claw
    Servo claw;

    // Torque servo that moves the arm up and down
    Servo arm;

    // Torque servo that rotates the arm
    Servo rotate;


    // =================================================
    // CLAW POSITIONS
    // =================================================

    static final double CLAW_OPEN = 0.20;
    static final double CLAW_CLOSE = 0.50;


    // =================================================
    // ARM POSITIONS
    // =================================================

    static final double ARM_DOWN = 0.20;
    static final double ARM_UP = 0.40;


    // =================================================
    // ROTATION SETTINGS
    // =================================================

    // Starting position of the rotate servo
    double rotatePosition = 0.50;

    // How far the servo moves each button press
    static final double ROTATE_STEP = 0.10;

    // Minimum and maximum rotation positions
    static final double ROTATE_MIN = 0.10;
    static final double ROTATE_MAX = 0.90;

    // Used to detect a NEW button press
    boolean lastLeftBumper = false;
    boolean lastRightBumper = false;


    @Override
    public void runOpMode() {

        // =================================================
        // CONNECT SERVOS
        // =================================================

        // These names MUST exactly match your Robot Configuration
        claw = hardwareMap.servo.get("claw");
        arm = hardwareMap.servo.get("arm");
        rotate = hardwareMap.servo.get("rotate");


        // =================================================
        // STARTING POSITIONS
        // =================================================

        // Start claw open
        claw.setPosition(CLAW_OPEN);

        // Start arm down
        arm.setPosition(ARM_DOWN);

        // Start rotate servo at 0.50
        rotate.setPosition(rotatePosition);


        // =================================================
        // WAIT FOR START
        // =================================================

        waitForStart();


        // =================================================
        // MAIN LOOP
        // =================================================

        while (opModeIsActive()) {


            // =================================================
            // CLAW
            // =================================================

            // A = close claw
            if (gamepad2.a) {
                claw.setPosition(CLAW_CLOSE);
            }

            // B = open claw
            if (gamepad2.b) {
                claw.setPosition(CLAW_OPEN);
            }


            // =================================================
            // ARM
            // =================================================

            // X = move arm up
            if (gamepad2.x) {
                arm.setPosition(ARM_UP);
            }

            // Y = move arm down
            if (gamepad2.y) {
                arm.setPosition(ARM_DOWN);
            }


            // =================================================
            // ROTATE ARM
            // =================================================

            // RIGHT BUMPER = move one step RIGHT
            // Only happens once per button press
            if (gamepad2.right_bumper && !lastRightBumper) {

                rotatePosition += ROTATE_STEP;

                // Prevent the servo from going past the limit
                if (rotatePosition > ROTATE_MAX) {
                    rotatePosition = ROTATE_MAX;
                }

                rotate.setPosition(rotatePosition);
            }


            // LEFT BUMPER = move one step LEFT
            // Only happens once per button press
            if (gamepad2.left_bumper && !lastLeftBumper) {

                rotatePosition -= ROTATE_STEP;

                // Prevent the servo from going past the limit
                if (rotatePosition < ROTATE_MIN) {
                    rotatePosition = ROTATE_MIN;
                }

                rotate.setPosition(rotatePosition);
            }


            // Remember whether the buttons were pressed
            // this loop so we can detect the next press
            lastRightBumper = gamepad2.right_bumper;
            lastLeftBumper = gamepad2.left_bumper;


            // =================================================
            // TELEMETRY
            // =================================================

            telemetry.addData("Claw Position", claw.getPosition());
            telemetry.addData("Arm Position", arm.getPosition());
            telemetry.addData("Rotate Position", rotatePosition);

            telemetry.addLine("");
            telemetry.addLine("A = Close Claw");
            telemetry.addLine("B = Open Claw");
            telemetry.addLine("X = Arm Up");
            telemetry.addLine("Y = Arm Down");
            telemetry.addLine("LB = Rotate Left");
            telemetry.addLine("RB = Rotate Right");

            telemetry.update();
        }
    }
}