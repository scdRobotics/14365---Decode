package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

//@TeleOp
public class Intake extends LinearOpMode
{

    ArrayList<Integer> balls = new ArrayList<>();
    ArrayList<Integer> order = new ArrayList<>();

    int shootingPosition = 1;

    float red;
    float green;
    float blue;

    int orderLocation;

    private NormalizedColorSensor colorSensor;

    DcMotor shootMotor, intakeMotor;
    Servo servoBottom, servoTop;
    final float bottomDownPos = 0.5f;
    final float bottomUpPos = 0.55f;
    final float topClosePos = 0.75f;
    final float topOpenPos = 0.9f;

    public Intake(@NonNull HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        order.add(0);
        order.add(0);
        order.add(0);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "internalColorSensor");

        balls.add(0);
        balls.add(0);
        balls.add(0);

        shootMotor = hardwareMap.dcMotor.get("shootMotor");

        servoBottom = hardwareMap.get(Servo.class, "bottomIntakeServo");
        servoTop = hardwareMap.get(Servo.class, "topIntakeServo");
    }
    public void init(@NonNull ArrayList<Integer> goodOrder)
    {
        order.set(0, goodOrder.get(0));
        order.set(1, goodOrder.get(1));
        order.set(2, goodOrder.get(2));
    }

    @Override
    public void runOpMode()
    {
        /*
        order.add(2);
        order.add(1);
        order.add(1);

        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "internalColorSensor");
        waitForStart();
        if(isStopRequested()) return;

        boolean lastFrameLeft = false;
        boolean lastFrameRight = false;
        boolean lastFrameRightTrigger = false;

        balls.add(0);
        balls.add(0);
        balls.add(0);
        */

        while(opModeIsActive())
        {
            red = colorSensor.getNormalizedColors().red;
            blue = colorSensor.getNormalizedColors().blue;
            green = colorSensor.getNormalizedColors().green;

            if(green > red + .001 && green > blue + .001 && green > 0.002)
            {
                logBalls();
                balls.set(0, 2);
            }
            if(blue > red + 0.001 && blue > green + 0.001 && blue > 0.002)
            {
                logBalls();
                balls.set(0,1);
            }

            /*if(gamepad1.dpad_right && !lastFrameRight)
            {
                rotateCounterClockwise();
            }
            if(gamepad1.dpad_left && !lastFrameLeft)
            {
                rotateClockwise();
            }
            if(gamepad1.right_trigger != 0 && !lastFrameRightTrigger)
            {
                telemetry.clear();
                orderedShoot(0);
                logBalls();
                telemetry.update();
            }

            telemetry.update();

            lastFrameRight = gamepad1.dpad_right;
            lastFrameLeft = gamepad1.dpad_left;
            lastFrameRightTrigger = gamepad1.right_trigger != 0;*/
        }
    }


    public void update()
    {
        red = colorSensor.getNormalizedColors().red;
        blue = colorSensor.getNormalizedColors().blue;
        green = colorSensor.getNormalizedColors().green;

        if(green > red + .001 && green > blue + .001 && green > 0.002)
        {
            balls.set(0, 2);
        }
        if(blue > red + 0.001 && blue > green + 0.001 && blue > 0.002)
        {
            balls.set(0,1);
        }
        //if(gamepad1.dpad_left) incrementOrderLocationDown();
        //if(gamepad1.dpad_right) incrementOrderLocationUp();
        logBalls();
    }
    public boolean isEmpty()
    {
        return balls.get(0) == 0 && balls.get(1) == 0 && balls.get(2) == 0;
    }
    public int greenCollected()
    {
        int count = 0;

        for(int i = 0; i < balls.size(); i++)
        {
            if(balls.get(i) == 2) count++;
        }
        return count;
    }
    public int purpleCollected()
    {
        int count = 0;

        for(int i = 0; i < balls.size(); i++)
        {
            if(balls.get(i) == 1) count++;
        }
        return count;
    }

    public void incrementOrderLocationUp()
    {
        orderLocation++;
        orderLocation %= 3;
    }
    /*public void incrementOrderLocationDown()
    {
        orderLocation--;
        orderLocation = (orderLocation < 0 ? 2: orderLocation);
    }*/

    public void orderedShoot(double vel)
    {
        if(vel == -100)
        {
            telemetry.addLine("calculations failed :(");
            return;
        }
        int purple = purpleCollected();
        int green = greenCollected();

        while(!isEmpty())
        {
            int colorToShoot = order.get(orderLocation);

            if(balls.get(shootingPosition) == colorToShoot)
            {
                shoot((float)vel);
            }
            else if(balls.get((shootingPosition+1)%balls.size()) == colorToShoot)
            {
                rotateClockwise();
                shoot((float)vel);
            }
            else if(balls.get(shootingPosition-1 < 0 ? 2: shootingPosition-1) == colorToShoot)
            {
                rotateCounterClockwise();
                shoot((float)vel);
            }
            else return;

            incrementOrderLocationUp();
        }
    }

    //R^2 = 0.967
    public float getPower(float dist)
    {
        return (float)(-3*Math.pow(10,-8)*Math.pow(dist,4) + 7*Math.pow(10,-6)*Math.pow(dist,3) - 0.0005*Math.pow(dist,2) + 0.0193*dist + 0.4762);
    }

    //R^2 = 0.9593
    public float getPowLinear(float dist)
    {
        return (float)(0.0027*dist + 0.6567);
    }

    public void shoot(float power)
    {
        //spin shootMotor
        shootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor.setPower(-power);
        //wait seconds
        sleepTime(4000);
        loadNextBall();
    }
    public void shoot3(float power)
    {
        shootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor.setPower(-power);
        sleepTime(4000);
        loadNextBall();
        sleepTime(250);
        loadNextBall();
        sleepTime(250);
        loadNextBall();

        shootMotor.setPower(0);
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
        while(System.currentTimeMillis() - startTime < ms);
    }

    public void openBottomServo(boolean open)
    {
        servoBottom.setPosition(open ? bottomUpPos : bottomDownPos);
    }
    public void openTopServo(boolean open)
    {
        servoTop.setPosition(open ? topOpenPos : topClosePos);
    }

    public void rotateCounterClockwise()
    {
        telemetry.addLine("turn counterclockwise");
        ArrayList<Integer> newBalls = new ArrayList<>();

        newBalls.add(0);
        newBalls.add(0);
        newBalls.add(0);

        for(int i = 0; i < balls.size(); i++)
        {
            newBalls.set( (i+1)%balls.size(), balls.get(i) );
        }
        balls = newBalls;
    }
    public void rotateClockwise()
    {
        telemetry.addLine("turn clockwise");
        ArrayList<Integer> newBalls = new ArrayList<>();

        newBalls.add(0);
        newBalls.add(0);
        newBalls.add(0);

        for(int i = 0; i < balls.size(); i++)
        {
            newBalls.set( (i-1 < 0 ? 2: i-1), balls.get(i) );
        }
        balls = newBalls;
    }
    private void logBalls()
    {
        telemetry.addData("red", red);
        telemetry.addData("blue", blue);
        telemetry.addData("green", green);

        telemetry.addLine("\nhave balls:");
        for(int i : balls)
        {
            if(i == 0) telemetry.addLine("none");
            else if(i == 1) telemetry.addLine("purple");
            else telemetry.addLine("green");
        }
        telemetry.addLine("\ncorrect order:");
        for(int i = 0; i < order.size(); i++)
        {
            if(order.get(i) == 1) telemetry.addLine("purple" + (i == orderLocation ? " <" : ""));
            else if(order.get(i) == 2) telemetry.addLine("green" + (i == orderLocation ? " <" : ""));
            else telemetry.addLine("nothing");
        }
    }
}