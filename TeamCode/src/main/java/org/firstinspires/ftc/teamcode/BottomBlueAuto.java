package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BottomBlueAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotController rc = new RobotController(hardwareMap, telemetry, "blue");

        waitForStart();

        rc.moveBackward(48, 0.4f);
        rc.moveLeft(6,0.2f);
        rc.moveBackward(8,0.4f);
        rc.turnToCenterGoal(0.4f, 3);
        rc.shoot3(1400);
        rc.moveLeft(12, 0.5f);
    }
}