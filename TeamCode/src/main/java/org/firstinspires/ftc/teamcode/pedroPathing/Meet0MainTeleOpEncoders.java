package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// needs the encoder cables from outtake1 and outtake2 plugged into the hub
@TeleOp(name = "Meet0 #2 MAIN TeleOp + encoders ★", group = "1 Meet 0")
public class Meet0MainTeleOpEncoders extends Meet0MainTeleOp {

    @Override
    protected boolean useEncoders() {
        return true;
    }
}
