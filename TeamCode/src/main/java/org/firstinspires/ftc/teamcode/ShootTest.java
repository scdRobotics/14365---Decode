package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ShootTest extends LinearOpMode {
    final float interval = 0.05f;
    float power = 1;
    float bottomServoExtendedPos = 0.5f;
    float topServoExtendedPos = 0.6f;

    DcMotor shootMotor; //Ticks Per Rotation = 28
    Servo servoTop, servoBottom;

    IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        shootMotor = hardwareMap.dcMotor.get("shootMotor");
        shootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        servoBottom = hardwareMap.get(Servo.class, "bottomIntakeServo");
        servoTop = hardwareMap.get(Servo.class, "topIntakeServo");
        imu = hardwareMap.get(IMU.class, "imu");
        Intake intake = new Intake(hardwareMap, telemetry);

        AprilTagDetection aprilTagDetection = new AprilTagDetection(hardwareMap, telemetry);

        aprilTagDetection.initAprilTag();

        boolean lastFrameDPad = false;
        boolean lastFrameB = false;
        boolean lastFrameX = false;
        boolean lastFrameY = false;

        servoBottom.setPosition(0.5);
        servoTop.setPosition(0.75);

        while(opModeIsActive())
        {
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
            if(power < 0) power = 0;
            if(power > 1) power = 1;
            telemetry.addLine("press dpad up/down to increase/decrease power");
            telemetry.addLine("hold 'a' to spin shootMotor \n");

            telemetry.addData("bottom servo position", bottomServoExtendedPos);
            telemetry.addData("top servo position", topServoExtendedPos);
            telemetry.addData("shootPosition", shootMotor.getCurrentPosition());

            telemetry.addData("power", power);
            telemetry.addData("calculated Distance", aprilTagDetection.getDistance());
            if(gamepad1.a) shootMotor.setPower(-power);
            else shootMotor.setPower(0);

            if(gamepad1.b && !lastFrameB)
            {
                shoot(intake.getPowLinear((float)aprilTagDetection.getDistance()));
                lastFrameB = true;
            }
            if(gamepad1.x && !lastFrameX)
            {
                shoot(intake.getPower((float)aprilTagDetection.getDistance()));
                lastFrameX = true;
            }
            if(gamepad1.y && !lastFrameY)
            {
                shoot(power);
                lastFrameY = true;
            }

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
            if(!lastFrameY) lastFrameY = false;
        }
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
        sleepTime(500);
        loadNextBall();
        sleepTime(500);
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
        servoBottom.setPosition(open ? 0.55 : 0.5);
    }
    public void openTopServo(boolean open)
    {
        servoTop.setPosition(open ? 0.9 : 0.75);
    }
}