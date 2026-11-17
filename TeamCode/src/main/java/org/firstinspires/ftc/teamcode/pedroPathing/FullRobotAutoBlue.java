package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Full Robot Auto BLUE - Shoot + Park", group = "0 Full Robot", preselectTeleOp = "Full Robot #1 TeleOp ★")
public class FullRobotAutoBlue extends FullRobotAutoRed {

    @Override
    protected String alliance() {
        return "BLUE";
    }
}
