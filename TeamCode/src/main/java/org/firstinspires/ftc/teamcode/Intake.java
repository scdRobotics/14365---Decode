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
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

//@TeleOp
public class Intake extends LinearOpMode
{
    int shootingPosition = 1;

    private NormalizedColorSensor colorSensor;

     DcMotorEx shootMotor;
    Servo servoBottom, servoTop;
    final float bottomClosePos = 0.45f;
    final float bottomOpenPos = 0.55f;
    final float topClosePos = 0.7f;
    final float topOpenPos = 0.9f;
    //dist, power
    private Map<Integer, Integer> powers = new HashMap<>();

    public Intake(HardwareMap hardwareMap, Telemetry telemetry)
    {
        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");

        servoBottom = hardwareMap.get(Servo.class, "bottomIntakeServo");
        servoTop = hardwareMap.get(Servo.class, "topIntakeServo");

        powers.put(94,1850);
        powers.put(93, 1850);
        powers.put(92,1850);
        powers.put(91, 1800);
        powers.put(90, 1800);
        powers.put(89, 1750);
        powers.put(88, 1750);
        powers.put(87, 1725);
        powers.put(86, 1725);
        powers.put(85,1725);
        powers.put(84, 1700);
        powers.put(83, 1700);
        powers.put(82, 1700);
        powers.put(81, 1700);
        powers.put(80, 1700);
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
        float x = dist; //12393.71 - 915.885*x + 27.84441*Math.pow(x,2) - 0.3649013*Math.pow(x,3) + 0.001754586*Math.pow(x,4)
        if(x < 60) return (float)(12333.71 - 915.885*x + 27.84441*Math.pow(x,2) - 0.3649013*Math.pow(x,3) + 0.001754586*Math.pow(x,4));
        else
        {
            if(powers.get((int)Math.ceil(dist)) == null || powers.get((int)Math.floor(dist)) == null)
            {
                if(Math.ceil(dist) < 80) return 1675;
                else return 1900;
            }
            else
            {
                float power = (powers.get((int)Math.ceil(dist)) + powers.get((int)Math.floor(dist))) / 2;
                return power;
            }
//            float power = (float)(-101683.3 + 3852.621*x - 47.82096*Math.pow(x,2) + 0.19780809*Math.pow(x,3));
//            return Math.max(1730, power);
        }
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

        sleepTime(4000);
        loadNextBall();
    }
    public void shoot3Timed(float power)
    {
        shootMotor.setVelocity(-power);
        sleepTime(5000);
        loadNextBall();
        sleepTime(1300);
        loadNextBall();
        sleepTime(1300);
        loadNextBall();
        openTopServo(true);

        shootMotor.setVelocity(0);
    }
    public void shoot3(float power)
    {
        shootMotor.setVelocity(-power);
        waitForVelo(power);
        loadNextBall();
        waitForVelo(power);
        loadNextBall();
        waitForVelo(power);
        loadNextBall();
        openTopServo(true);
    }
    void loadNextBall()
    {
        openBottomServo(true);
        //wait less than second
        sleepTime(500);
        //close servoBottom
        openBottomServo(false);
        sleepTime(75);
        dropBall();
    }
    void dropBall()
    {
        openTopServo(true);
        //wait <second
        sleepTime(500);
        //close servoTop
        openTopServo(false);
    }
    private void waitForVelo(float power)
    {
        long startTime = System.currentTimeMillis();
        while(Math.abs(shootMotor.getVelocity()) < Math.min(1780,power) && System.currentTimeMillis() - startTime < 3000)
        {

        }
        sleepTime(500);
    }

    private void sleepTime(long ms)
    {
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime < ms)
        {

        }
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