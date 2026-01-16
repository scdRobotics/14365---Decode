package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class TopRedAuto extends LinearOpMode {
    public void runOpMode()
    {
        RobotController rc = new RobotController(hardwareMap, telemetry, "red");
<<<<<<< Updated upstream

        waitForStart();
=======
        //Odometry odometry = new Odometry(hardwareMap, telemetry, "blue");
        rc.odometry.setPose(Odometry.topRedStart);
>>>>>>> Stashed changes

        waitForStart();
        rc.shoot3(1700);
        rc.intake.shootMotor.setVelocity(0);
        rc.moveForward(20, 0.4f);
    }
}