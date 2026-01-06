package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class TopRedAuto  extends LinearOpMode {
    public void runOpMode()
    {
        RobotController rc = new RobotController(hardwareMap, telemetry, "red");

        waitForStart();

        rc.moveForward(5, 0.4f);
        rc.turnToCenterGoal(.4f, 5, -218);
        rc.shoot3(rc.getPow()-15);
        rc.moveForward(20, 0.4f);
    }
}