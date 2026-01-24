package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class TopRedAuto extends LinearOpMode {

    public void runOpMode() {
        RobotController rc = new RobotController(hardwareMap, telemetry, "red");

        waitForStart();
        rc.shoot3(1725);
        rc.intake.shootMotor.setVelocity(0);
        rc.moveForward(0.5f, 0.6f);
    }
}
