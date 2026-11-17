package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Arrays;

@TeleOp(name = "Tool - AprilTag Check (Limelight)", group = "3 Tools")
public class AprilTagCheck extends LinearOpMode {

    // the 16 goal tags, four per hive cell
    static final int FIRST_TAG = 30;
    static final String[] COLORS = {"red", "red", "blue", "blue"};
    static final String[] SIDES = {"scoring", "audience", "audience", "scoring"};

    @Override
    public void runOpMode() {
        // name has to match the robot config on the driver hub
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // apriltag pipeline number from the limelight web page (36h11, tag size 82.55 mm, full 3d on)
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.setMsTransmissionInterval(11);

        boolean[] found = new boolean[16];

        while (opModeInInit()) {
            check(limelight, found);
        }

        while (opModeIsActive()) {
            check(limelight, found);
        }

        limelight.stop();
    }

    private void check(Limelight3A limelight, boolean[] found) {
        if (gamepad1.a) {
            Arrays.fill(found, false);
        }

        LLStatus status = limelight.getStatus();
        telemetry.addData("limelight", "%s  pipeline %d %s  %.0f fps",
                limelight.isConnected() ? "connected" : "not connected",
                status.getPipelineIndex(), status.getPipelineType(), status.getFps());

        int notOnAGoal = 0;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getStaleness() < 200) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                int id = tag.getFiducialId();
                if (id < FIRST_TAG || id >= FIRST_TAG + 16) {
                    notOnAGoal++;
                    continue;
                }
                int cell = (id - FIRST_TAG) / 4;
                found[id - FIRST_TAG] = true;
                Position p = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
                double distance = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                telemetry.addData("tag " + id, "%s, %s side   tx %.1f   ty %.1f   %.1f in",
                        COLORS[cell], SIDES[cell], tag.getTargetXDegrees(), tag.getTargetYDegrees(), distance);
            }
        }

        int count = 0;
        for (boolean seen : found) {
            if (seen) count++;
        }

        telemetry.addLine();
        telemetry.addData("found", "%d of 16   (a clears the list)", count);
        telemetry.addData("tags not on a goal", notOnAGoal);
        for (int cell = 0; cell < 4; cell++) {
            StringBuilder line = new StringBuilder();
            for (int i = 0; i < 4; i++) {
                int id = FIRST_TAG + cell * 4 + i;
                line.append(id).append(found[id - FIRST_TAG] ? " yes   " : " no    ");
            }
            telemetry.addData(COLORS[cell] + " " + SIDES[cell], line.toString());
        }
        telemetry.update();
    }
}
