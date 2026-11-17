package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Full Robot #2 TeleOp + encoders ★", group = "0 Full Robot")
public class FullRobotTeleOpEncoders extends FullRobotTeleOp {

    // needs the encoder cables on outtake1 and outtake2
    @Override
    protected boolean useEncoders() {
        return true;
    }
}
