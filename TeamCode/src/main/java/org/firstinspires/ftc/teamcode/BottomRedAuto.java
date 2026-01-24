package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BottomRedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotController rc = new RobotController(hardwareMap, telemetry, "red");

        waitForStart();

        rc.moveBackward(600f, 0.5f);
        rc.shoot3(1400);
        rc.moveRight(500f, 0.5f);
    }
}
