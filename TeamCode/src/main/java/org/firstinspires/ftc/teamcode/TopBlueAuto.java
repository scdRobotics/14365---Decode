package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class TopBlueAuto extends LinearOpMode {

    public void runOpMode() {
        RobotController rc = new RobotController(hardwareMap, telemetry, "blue");
        //Odometry odometry = new Odometry(hardwareMap, telemetry, "blue");
        rc.odometry.setPose(Odometry.topBlueStart);
        telemetry.addData("pose", "(" + rc.odometry.getPose().x + ", " + rc.odometry.getPose().y + ", " + rc.odometry.getPose().angle + ")");
        telemetry.addData("angle to goal", rc.odometry.getAngleToGoal());
        telemetry.update();
        waitForStart();

        rc.shoot3(1725);
        rc.intake.shootMotor.setVelocity(0);
        rc.moveForward(0.5f, 0.6f);
    }
}
