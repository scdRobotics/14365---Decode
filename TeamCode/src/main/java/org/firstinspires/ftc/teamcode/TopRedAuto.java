package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class TopRedAuto extends LinearOpMode {

    public void runOpMode() {
        RobotController rc = new RobotController(hardwareMap, telemetry, "red");

        waitForStart();
        rc.shoot3(1725);
        rc.intake.shootMotor.setVelocity(0);
        rc.moveBackward(500f, .5f);
        /*for(DcMotor motor : rc.motors)
        {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(1);
        }
        rc.motors.get(2).setPower(-1);*/
        /*rc.motors.get(0).setPower(1);
        rc.motors.get(1).setPower(1);

        rc.motors.get(3).setPower(1);*/
        while(!isStopRequested());
    }
}
