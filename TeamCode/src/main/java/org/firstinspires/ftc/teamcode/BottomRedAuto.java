package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BottomRedAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotController rc = new RobotController(hardwareMap, telemetry);

        waitForStart();

        rc.moveToGoalDistance(36, .5f, 3);
        rc.turnToCenterGoal(0.5f, 5);
        rc.shoot3(0.7775f);
        rc.moveLeft(200, 0.5f);
    }
}
