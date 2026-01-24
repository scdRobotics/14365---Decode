package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BottomBlueAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotController rc = new RobotController(hardwareMap, telemetry, "blue");

        waitForStart();

        rc.moveBackward(0.6f, 0.5f);
        rc.shoot3(1400);
        rc.moveLeft(0.5f, 0.5f);
    }
}
