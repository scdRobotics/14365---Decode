package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BottomBlueAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotController rc = new RobotController(hardwareMap, telemetry);

        waitForStart();

        rc.moveBackward(48, 0.4f);
        rc.moveLeft(6,0.2f);
        rc.moveBackward(8,0.4f);
        rc.shoot3(0.8f);
        rc.moveLeft(6, 0.5f);
    }
}
