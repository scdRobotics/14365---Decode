package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceTest extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        waitForStart();


        while(!isStopRequested())
        {
            telemetry.addData("Distance reported", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
