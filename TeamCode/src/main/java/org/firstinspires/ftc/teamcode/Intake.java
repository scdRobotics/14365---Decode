package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

//@TeleOp
public class Intake extends LinearOpMode
{
    int shootingPosition = 1;

    private NormalizedColorSensor colorSensor;

     DcMotorEx shootMotor;
    Servo servoBottom, servoTop;
    final float bottomClosePos = 0.5f;
    final float bottomOpenPos = 0.55f;
    final float topClosePos = 0.75f;
    final float topOpenPos = 0.9f;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry)
    {
        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");

        servoBottom = hardwareMap.get(Servo.class, "bottomIntakeServo");
        servoTop = hardwareMap.get(Servo.class, "topIntakeServo");
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

    //R^2 = 0.967
    public float getPower(float dist)
    {
        return (float)(-3*Math.pow(10,-8)*Math.pow(dist,4) + 7*Math.pow(10,-6)*Math.pow(dist,3) - 0.0005*Math.pow(dist,2) + 0.0193*dist + 0.4762);
    }

    public float getPolynomialPower(float dist)
    {
        float x = dist;
        if(x < 65) return (float)(12393.71 - 915.885*x + 27.84441*Math.pow(x,2) - 0.3649013*Math.pow(x,3) + 0.001754586*Math.pow(x,4));
        else return (float)(-101683.3 + 3852.621*x - 47.82096*Math.pow(x,2) + 0.19780809*Math.pow(x,3));

    }

    //R^2 = 0.9593
    public float getPowLinear(float dist)
    {
        return (float)(0.0027*dist + 0.6567);
    }

    public void shoot(float power)
    {
        //spin shootMotor
        shootMotor.setVelocity(-power);
        //wait seconds

        telemetry.addData("velocity", shootMotor.getVelocity());
        telemetry.update();

        sleepTime(4000);
        loadNextBall();
    }
    public void shoot3(float power)
    {
        shootMotor.setVelocity(-power);
        telemetry.addData("velocity", shootMotor.getVelocity());
        telemetry.update();
        sleepTime(4000);
        telemetry.addData("velocity", shootMotor.getVelocity());
        telemetry.update();
        loadNextBall();
        sleepTime(1500);
        telemetry.addData("velocity", shootMotor.getVelocity());
        telemetry.update();
        loadNextBall();
        sleepTime(1500);
        telemetry.addData("velocity", shootMotor.getVelocity());
        telemetry.update();
        loadNextBall();

        shootMotor.setVelocity(0);
    }
    void loadNextBall()
    {
        openBottomServo(true);
        //wait less than second
        sleepTime(500);
        //close servoBottom
        openBottomServo(false);
        sleepTime(75);
        //open servoTop
        openTopServo(true);
        //wait <second
        sleepTime(500);
        //close servoTop
        openTopServo(false);
    }

    private void sleepTime(long ms)
    {
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime < ms)
        {
            telemetry.addData("velocity", shootMotor.getVelocity());
            telemetry.update();
        };
    }

    public void openBottomServo(boolean open)
    {
        servoBottom.setPosition(open ? bottomOpenPos : bottomClosePos);
    }
    public void openTopServo(boolean open)
    {
        servoTop.setPosition(open ? topOpenPos : topClosePos);
    }
}