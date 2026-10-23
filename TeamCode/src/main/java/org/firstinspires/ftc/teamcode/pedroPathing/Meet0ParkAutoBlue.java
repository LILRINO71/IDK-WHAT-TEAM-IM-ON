package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Meet0 Auto BLUE - Leave + Park", group = "1 Meet 0", preselectTeleOp = "Meet0 #1 MAIN TeleOp ★")
public class Meet0ParkAutoBlue extends Meet0ParkAutoRed {

    @Override
    protected String alliance() {
        return "BLUE";
    }
}
