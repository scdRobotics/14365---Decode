package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class TopRedAuto  extends LinearOpMode {
    public void runOpMode()
    {
        RobotController rc = new RobotController(hardwareMap, telemetry);

        waitForStart();

        rc.moveBackward(100, 0.5f);
        rc.turnToCenterGoal(0.5f, 5);
        rc.shoot3(rc.getPowLinear());
        rc.turnToAngle(0, .5f, 5);
        rc.moveLeft(2000, .5f);
    }
}