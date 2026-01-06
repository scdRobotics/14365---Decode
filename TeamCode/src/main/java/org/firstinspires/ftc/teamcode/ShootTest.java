package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Dictionary;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;

@TeleOp
public class ShootTest extends LinearOpMode {
    final float interval = 25f;
    final int TPR = 28;
    float power = 900;
    float bottomServoExtendedPos = 0.5f;
    float topServoExtendedPos = 0.6f;

    DcMotorEx shootMotor; //Ticks Per Rotation = 28
    Servo servoTop, servoBottom;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        //shootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        servoBottom = hardwareMap.get(Servo.class, "bottomIntakeServo");
        servoTop = hardwareMap.get(Servo.class, "topIntakeServo");
        imu = hardwareMap.get(IMU.class, "imu");
        Intake intake = new Intake(hardwareMap, telemetry);

        AprilTagDetection aprilTagDetection = new AprilTagDetection(hardwareMap, telemetry);

        aprilTagDetection.initAprilTag();

        Map<Integer, Integer> powers = new HashMap<>();
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

        boolean lastFrameDPad = false;
        boolean lastFrameB = false;
        boolean lastFrameX = false;
        boolean lastFrameY = false;

        long lastFrameShootMotorPos = 0;
        long lastFrameTime = 0;

        long deltaTime = 0;
        long deltaShootMotorPos = 0;

        float dist = 0;

        servoBottom.setPosition(intake.bottomClosePos);
        servoTop.setPosition(intake.topClosePos);

        while(opModeIsActive())
        {
            telemetry.addData("system time", System.currentTimeMillis());
            telemetry.addData("last frame time", lastFrameTime);
            deltaTime = System.currentTimeMillis()-lastFrameTime;
            deltaShootMotorPos = shootMotor.getCurrentPosition() - lastFrameShootMotorPos;
            if(deltaTime!=0)
            {
                double RPM = deltaShootMotorPos / deltaTime / TPR * 60000;
                telemetry.addData("RPM", RPM);
            }
            else telemetry.addLine("deltaTime is 0!!!");

            aprilTagDetection.telemetryAprilTag();

            if(gamepad1.dpad_up && !lastFrameDPad)
            {
                power+=interval;
                lastFrameDPad = true;
            }
            if(gamepad1.dpad_down && !lastFrameDPad)
            {
                power-=interval;
                lastFrameDPad = true;
            }

            telemetry.addLine("press dpad up/down to increase/decrease power");
            telemetry.addLine("hold 'a' to spin shootMotor \n");

            telemetry.addData("bottom servo position", bottomServoExtendedPos);
            telemetry.addData("top servo position", topServoExtendedPos);
            telemetry.addData("shootPosition", shootMotor.getCurrentPosition());

            telemetry.addData("power", power);
            telemetry.addData("velocity", shootMotor.getVelocity());
            telemetry.addData("calculated Distance", aprilTagDetection.getDistance());
            if(gamepad1.a) shootMotor.setPower(-power);
            else shootMotor.setPower(0);

            if(aprilTagDetection.getDistance() != -Double.MAX_VALUE) dist = (float)aprilTagDetection.getDistance();
            telemetry.addData("polynomial power", intake.getPolynomialPower(dist));

            if(gamepad1.b && !lastFrameB)
            {
                intake.shoot3(power);
                lastFrameB = true;
            }
            if(gamepad1.x && !lastFrameX)
            {
                intake.shoot3(intake.getPolynomialPower(dist));
                lastFrameX = true;
            }
            if(gamepad1.y && !lastFrameY && dist != -Double.MAX_VALUE)
            {
                if(powers.get((int)Math.ceil(dist)) == null || powers.get((int)Math.floor(dist)) == null)
                {
                    if(Math.ceil(dist) < 80) intake.shoot(1675);
                }
                else
                {
                    float power = (powers.get((int)Math.ceil(dist)) + powers.get((int)Math.floor(dist))) / 2;
                    intake.shoot(power);
                    telemetry.addData("hashmap power", power);
                    lastFrameY = true;
                }
            }
            if(gamepad1.right_bumper) intake.loadNextBall();

            //servoBottom.setPosition(bottomServoExtendedPos);
            //servoTop.setPosition(topServoExtendedPos);

            //bottom: down 0.5, up 0.55
            //top: close: 0.75 open: 0.9

            telemetry.addData("yaw angle", imu.getRobotYawPitchRollAngles().getYaw());

            telemetry.addData("lastFrameDPad", lastFrameDPad);
            telemetry.addData("DPAD up", gamepad1.dpad_up);
            telemetry.addData("DPAD down", gamepad1.dpad_down);

            telemetry.update();

            if(!gamepad1.dpad_down && !gamepad1.dpad_up) lastFrameDPad = false;
            if(!gamepad1.b) lastFrameB = false;
            if(!gamepad1.x) lastFrameX = false;
            if(!gamepad1.y) lastFrameY = false;

            lastFrameShootMotorPos = shootMotor.getCurrentPosition();
            lastFrameTime = System.currentTimeMillis();
        }
    }

    private void sleepTime(long ms)
    {
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime < ms);
    }

    public void openBottomServo(boolean open)
    {
        servoBottom.setPosition(open ? 0.55 : 0.5);
    }
    public void openTopServo(boolean open)
    {
        servoTop.setPosition(open ? 0.9 : 0.75);
    }

}