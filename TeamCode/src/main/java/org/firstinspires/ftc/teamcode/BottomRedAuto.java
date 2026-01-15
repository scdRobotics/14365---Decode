package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BottomRedAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotController rc = new RobotController(hardwareMap, telemetry, "red");
        rc.odometry.setPose(rc.odometry.bottomRedStart);

        waitForStart();

        rc.moveBackward(45, 0.4f);
        rc.moveRight(6,0.2f);
        rc.moveBackward(8,0.4f);
        rc.turnToCenterGoal(0.4f, 3, -100);
        telemetry.addData("velo to shoot at", rc.getPow());
        telemetry.update();
        rc.shoot3(rc.getPow());
        rc.moveRight(15, 0.5f);
    }
}
